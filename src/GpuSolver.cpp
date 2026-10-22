#include "stonefish_net/GpuSolver.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <utility>

#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <Stonefish/core/SimulationManager.h>
#include <Stonefish/entities/SolidEntity.h>
#include <Stonefish/entities/forcefields/Ocean.h>

#include "convert.h"
#include "pose_now.h"
#include "stonefish_net/core/coloring.hpp"
#include "stonefish_net/core/current_grid.hpp"

namespace stonefish_net
{
namespace
{
std::vector<core::Gripper> gripper_specs(const std::vector<GpuSolver::Gripper>& grippers)
{
  std::vector<core::Gripper> specs;
  for (const auto& g : grippers) specs.push_back(g.spec);
  return specs;
}

// El extremo de cada orinque, ya anclado a su boya: no se agarra
std::vector<int> buoy_nodes(const core::NetTopology& topo)
{
  std::vector<int> nodes;
  for (const auto& b : topo.marks.buoys) nodes.push_back(b.node);
  return nodes;
}

double ms_between(std::chrono::steady_clock::time_point a, std::chrono::steady_clock::time_point b)
{
  return std::chrono::duration<double, std::milli>(b - a).count();
}
}  // namespace

GpuSolver::GpuSolver(const core::NetTopology& topo, const core::NetConfig& cfg, sf::SimulationManager* sm,
                     SolidContacts& solids, std::vector<sf::SolidEntity*> buoys, const Eigen::Vector3d& eye,
                     SDL_Window* window, std::string shader_dir, std::vector<Gripper> grippers)
: topo_(topo),
  cfg_(cfg),
  sm_(sm),
  solids_(solids),
  buoys_(std::move(buoys)),
  eye_(eye),
  layout_(core::make_gpu_layout(topo, core::color_links(topo))),
  gl_(window, std::move(shader_dir)),
  grippers_(std::move(grippers)),
  grasp_(topo, gripper_specs(grippers_), buoy_nodes(topo)),
  diagnostics_(cfg.diagnostics.enabled ? std::make_unique<NetDiagnostics>() : nullptr)
{
  for (const auto& n : topo_.nodes) positions_.push_back(n.position.cast<float>());
}

void GpuSolver::start_gpu()
{
  try {
    gl_.ensure_current();
    xpbd_ = std::make_unique<gpu::GpuXpbd>(gl_, layout_);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "[stonefish_net] la red no puede calcularse en la GPU: %s; usa solver.backend: bullet\n", e.what());
    std::exit(1);
  }
  std::fprintf(stderr, "[stonefish_net] red en la GPU: %s, %zu nudos, %zu tramos en %d colores, %d subpasos por tick\n",
               gl_.vendor().c_str(), layout_.node_count(), layout_.link_count(), layout_.colors(), cfg_.solver.substeps);
}

Eigen::Vector3d GpuSolver::eye_of(size_t buoy) const
{
  return eig(o_now(buoys_[buoy]) * bt(eye_));
}

// Corriente del océano en una rejilla de lado hydro.current_grid sobre la caja de la red del tick anterior más una
// celda por cada lado (core::current_grid_for: acotada aunque un nudo se escape); la GPU la interpola en cada nudo.
// Sin océano, agua quieta.
void GpuSolver::sample_current()
{
  const sf::Ocean* ocean = sm_->isOceanEnabled() ? sm_->getOcean() : nullptr;
  if (ocean == nullptr) {
    xpbd_->set_current_grid(Eigen::Vector3f::Zero(), 1.f, 1, 1, 1, {Eigen::Vector3f::Zero()});
    return;
  }
  const core::CurrentGrid g = core::current_grid_for(positions_, static_cast<float>(cfg_.hydro.current_grid));
  if (g.cell > static_cast<float>(cfg_.hydro.current_grid) && !warned_grid_) {
    std::fprintf(stderr, "[stonefish_net] AVISO: la red ocupa demasiado (¿un nudo escapado?); corriente en celdas de %.2f m\n",
                 g.cell);
    warned_grid_ = true;
  }
  std::vector<Eigen::Vector3f> v;
  v.reserve(static_cast<size_t>(g.dims[0] * g.dims[1] * g.dims[2]));
  for (int iz = 0; iz < g.dims[2]; ++iz) {
    for (int iy = 0; iy < g.dims[1]; ++iy) {
      for (int ix = 0; ix < g.dims[0]; ++ix) {
        const Eigen::Vector3f p = g.origin + g.cell * Eigen::Vector3f(ix, iy, iz);
        v.push_back(eig(ocean->GetFluidVelocity(bt(p.cast<double>()))).cast<float>());
      }
    }
  }
  xpbd_->set_current_grid(g.origin, g.cell, g.dims[0], g.dims[1], g.dims[2], v);
}

// Agarra y suelta con los dedos al inicio del tick y la red del tick anterior
void GpuSolver::update_grasp()
{
  std::vector<core::Finger> fingers;
  for (size_t b = 0; b < solids_.bodies().size(); ++b) fingers.push_back({solids_.pose_start(b), solids_.bodies()[b].box});
  for (const auto& e : grasp_.update(positions_, fingers)) {
    const char* name = grippers_[e.gripper].name.c_str();
    if (e.released > 0) std::fprintf(stderr, "[stonefish_net] pinza %s: suelta %d nudos (total %d)\n", name, e.released, e.held);
    if (e.grabbed > 0) std::fprintf(stderr, "[stonefish_net] pinza %s: agarra %d nudos (total %d)\n", name, e.grabbed, e.held);
  }
}

void GpuSolver::begin_step(double)
{
  if (!xpbd_) {
    start_gpu();
  }
  solids_.apply_reactions();
  solids_.snapshot_start();
  eye_start_.resize(buoys_.size());
  for (size_t k = 0; k < buoys_.size(); ++k) eye_start_[k] = eye_of(k).cast<float>();
  sample_current();
  if (!grippers_.empty()) {
    update_grasp();
  }
}

// El ojal cede según la masa y la inercia del cuerpo de Bullet de la boya (con su masa añadida), como el ancla de
// Bullet
std::vector<core::Anchor> GpuSolver::buoy_anchors(std::vector<Eigen::Vector3d>& lever) const
{
  std::vector<core::Anchor> anchors;
  lever.resize(buoys_.size());
  for (size_t k = 0; k < buoys_.size(); ++k) {
    const btRigidBody* rb = buoys_[k]->getRigidBody();
    lever[k] = eye_of(k) - eig(rb->getCenterOfMassPosition());
    core::Anchor a{topo_.marks.buoys[k].node, eye_start_[k], eye_of(k).cast<float>()};
    a.inv_mass = core::point_inv_mass(1.0 / rb->getInvMass(), eig(rb->getInvInertiaTensorWorld()), lever[k]);
    anchors.push_back(a);
  }
  return anchors;
}

// Cada boya, ya: donde la red dejó su ojal y, en el ojal, con la velocidad de lo que este se movió en el tick, para
// empezar el tick siguiente igual que en la red (core::Anchor). Nada que no sea finito llega a Bullet: un cuerpo con
// NaN lo saca de la simulación («Overflow in AABB»).
void GpuSolver::move_buoys(const std::vector<core::Anchor>& anchors, const std::vector<Eigen::Vector3d>& lever, double dt)
{
  for (size_t k = 0; k < buoys_.size(); ++k) {
    btRigidBody* rb = buoys_[k]->getRigidBody();
    const Eigen::Vector3d offset = anchor_offset_[k].cast<double>();
    const Eigen::Vector3d end = anchors[k].to.cast<double>() + offset;
    const Eigen::Vector3d dv = (end - anchors[k].from.cast<double>()) / dt - eig(rb->getVelocityInLocalPoint(bt(lever[k])));
    const Eigen::Vector3d impulse = core::eye_impulse(anchors[k].inv_mass.cast<double>(), dv);
    if (!offset.allFinite() || !impulse.allFinite()) continue;
    rb->applyImpulse(bt(impulse), bt(lever[k]));
    btTransform cg = rb->getCenterOfMassTransform();  // también la de interpolación, de la que sale lo que se dibuja
    cg.setOrigin(cg.getOrigin() + bt(offset));
    rb->setCenterOfMassTransform(cg);
    rb->activate();
  }
}

void GpuSolver::end_step(double dt)
{
  const int n = cfg_.solver.substeps;
  std::vector<Eigen::Vector3d> lever;
  std::vector<core::Anchor> anchors = buoy_anchors(lever);
  // Detrás de las boyas, los nudos agarrados atados a su dedo; los dedos no tocan lo que sujeta su pinza
  std::vector<core::ContactBody> bodies = solids_.contact_bodies();
  std::vector<core::GripAnchor> grips;
  if (!grippers_.empty()) {
    std::vector<Eigen::Isometry3d> body_end;
    for (size_t b = 0; b < bodies.size(); ++b) body_end.push_back(bodies[b].pose1);
    grips = grasp_.anchors(body_end);
    for (const auto& g : grips) anchors.push_back(g.anchor);
    grasp_.exclude_held(bodies);
  }
  xpbd_->set_anchors(anchors);
  xpbd_->set_bodies(bodies, n);

  core::XpbdParams p{};
  p.h = static_cast<float>(dt / n);
  p.rho = static_cast<float>(cfg_.hydro.water_density);
  p.g = static_cast<float>(sm_->getGravity().length());
  p.cn = static_cast<float>(cfg_.hydro.cn);
  p.ct = static_cast<float>(cfg_.hydro.ct);
  p.compliance = static_cast<float>(cfg_.solver.compliance);
  p.damping = core::substep_damping(cfg_.solver.damping, n);
  p.friction = static_cast<float>(cfg_.solver.friction);
  p.floor_z = static_cast<float>(cfg_.mooring.floor_z);
  const auto issued = std::chrono::steady_clock::now();
  for (int i = 0; i < n; ++i) xpbd_->substep(p, i, n);
  const auto read = std::chrono::steady_clock::now();
  xpbd_->read_back(positions_, anchor_dx_, anchor_offset_, reactions_);
  const auto done = std::chrono::steady_clock::now();

  move_buoys(anchors, lever, dt);
  // Los cuerpos reciben en el tick siguiente lo contrario de lo que la red cambió su cantidad de movimiento; cada
  // dedo, además, el tirón de sus nudos agarrados en su punto atado
  solids_.set_reactions(reactions_, dt, n);
  for (size_t j = 0; j < grips.size(); ++j) {
    const Eigen::Vector3d pull = anchor_dx_[buoys_.size() + j].cast<double>();
    solids_.add_reaction(grips[j].body, core::reaction_force(pull, dt, n), grips[j].anchor.to.cast<double>());
  }
  if (diagnostics_) {
    sample_diagnostics(lever, dt, ms_between(issued, read), ms_between(read, done));
  } else if (xpbd_->lost_contacts() > 0) {
    std::fprintf(stderr, "[stonefish_net] AVISO: %u contactos de la red sin reacción en este tick (registro lleno)\n",
                 xpbd_->lost_contacts());
  }
}

void GpuSolver::sample_diagnostics(const std::vector<Eigen::Vector3d>& lever, double dt, double issue_ms, double readback_ms)
{
  std::vector<Eigen::Vector3d> eye_v;
  std::vector<double> w;
  for (size_t k = 0; k < buoys_.size(); ++k) {
    const btRigidBody* rb = buoys_[k]->getRigidBody();
    eye_v.push_back(eig(rb->getVelocityInLocalPoint(bt(lever[k]))));
    w.push_back(rb->getAngularVelocity().length());
  }
  diagnostics_->sample_buoys(eye_v, w);
  diagnostics_->add_gpu({xpbd_->gpu_ms(), issue_ms, readback_ms, layout_.colors(), xpbd_->lost_contacts()});
  diagnostics_->sample(dt, positions_, topo_, cfg_.mooring.floor_z);
}

}  // namespace stonefish_net
