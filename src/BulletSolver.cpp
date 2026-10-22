#include "stonefish_net/BulletSolver.h"

#include <chrono>
#include <utility>

#include <BulletDynamics/Featherstone/btMultiBodyLinkCollider.h>
#include <BulletSoftBody/btSoftBody.h>
#include <BulletSoftBody/btSoftMultiBodyDynamicsWorld.h>
#include <Stonefish/core/SimulationManager.h>
#include <Stonefish/entities/SolidEntity.h>
#include <Stonefish/entities/forcefields/Ocean.h>

#include "convert.h"
#include "stonefish_net/core/contact.hpp"
#include "stonefish_net/core/hydro.hpp"

namespace stonefish_net
{
namespace
{
double seconds_since(std::chrono::steady_clock::time_point start)
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}
}  // namespace

BulletSolver::BulletSolver(const core::NetTopology& topo, const core::NetConfig& cfg, sf::SimulationManager* sm,
                           sf::Entity* owner, SolidContacts& solids, const std::vector<sf::SolidEntity*>& buoys,
                           const std::vector<const btCollisionObject*>& ignored)
: topo_(topo),
  cfg_(cfg),
  sm_(sm),
  solids_(solids),
  buoys_(buoys),
  robot_contact_(solids, topo),
  sleep_(topo, cfg.sleep),
  diagnostics_(cfg.diagnostics.enabled ? std::make_unique<NetDiagnostics>() : nullptr)
{
  std::vector<btVector3> x(topo_.nodes.size());
  std::vector<btScalar> m(topo_.nodes.size());
  for (size_t k = 0; k < topo_.nodes.size(); ++k) {
    const auto& n = topo_.nodes[k];
    x[k] = bt(n.position);
    m[k] = n.fixed ? 0 : n.inertial_mass;
  }
  body_ = new btSoftBody(&sm->getSoftBodyWorldInfo(), static_cast<int>(x.size()), x.data(), m.data());
  btSoftBody::Material* mat = body_->m_materials[0];
  mat->m_kLST = 1;  // hilos casi inextensibles; sin enlaces de flexión ni cortadura
  mat->m_kAST = 1;
  mat->m_kVST = 1;
  for (const auto& l : topo_.links) {
    body_->appendLink(l.a, l.b, mat);  // longitud natural: la distancia inicial (= l.rest_length)
  }
  body_->updateConstants();

  body_->m_cfg.piterations = cfg_.solver.iterations;
  body_->m_cfg.kDF = cfg_.solver.friction;  // × rozamiento del rígido, que Stonefish deja a 0: sin efecto con sus sólidos
  body_->m_cfg.kDP = cfg_.solver.damping;
  body_->m_cfg.kDG = 0.0;  // el arrastre lo pone begin_step (Morison)
  body_->m_cfg.kLF = 0.0;
  body_->m_cfg.kCHR = cfg_.solver.contact_hardness;  // contacto con rígidos más blando: menos temblor
  body_->m_cfg.kKHR = 1.0;
  body_->m_cfg.kSHR = 1.0;
  body_->m_cfg.kAHR = 1.0;
  body_->m_cfg.collisions = btSoftBody::fCollision::SDF_RS;
  body_->setCollisionFlags(body_->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
  body_->getCollisionShape()->setMargin(cfg_.solver.margin);
  body_->setUserPointer(owner);
  body_->setActivationState(DISABLE_DEACTIVATION);

  sm->getDynamicsWorld()->addSoftBody(body_, sf::MASK_DYNAMIC,
                                      sf::MASK_GHOST | sf::MASK_STATIC | sf::MASK_DYNAMIC | sf::MASK_ANIMATED_COLLIDING);

  for (const btCollisionObject* obj : ignored) body_->m_collisionDisabledObjects.push_back(obj);
  for (const auto& b : solids_.bodies()) {
    for (const btCollisionObject* co : {static_cast<const btCollisionObject*>(b.solid->getRigidBody()),
                                        static_cast<const btCollisionObject*>(b.solid->getMultiBodyLinkCollider())}) {
      if (co != nullptr) body_->m_collisionDisabledObjects.push_back(co);
    }
  }
  for (size_t k = 0; k < buoys_.size(); ++k) {
    body_->appendAnchor(topo_.marks.buoys.at(k).node, buoys_[k]->getRigidBody(), true, 1.0);
  }
  copy_positions();
}

void BulletSolver::begin_step(double dt)
{
  const auto start = std::chrono::steady_clock::now();
  const double rho = cfg_.hydro.water_density;
  const double g = sm_->getGravity().length();
  const sf::Ocean* ocean = sm_->isOceanEnabled() ? sm_->getOcean() : nullptr;
  const size_t n = topo_.nodes.size();

  // Hasta el primer subpaso, Bullet resuelve la red con sus propias iteraciones; desde aquí, end_step.
  body_->m_cfg.piterations = 0;
  solids_.apply_reactions();
  sleep_.update(*body_, dt, robot_contact_.robot_spheres());
  const auto& active = sleep_.active_nodes();

  // Un solo campo de agua por subpaso: el arrastre de cada tramo y su límite por nudo usan el mismo. Un nudo
  // dormido conserva el de su último subpaso despierto (lo usa el tramo frontera que llega a él).
  water_.resize(n, Eigen::Vector3d::Zero());
  for (int k : active) {
    water_[static_cast<size_t>(k)] = ocean ? eig(ocean->GetFluidVelocity(body_->m_nodes[k].m_x)) : Eigen::Vector3d::Zero();
  }

  // Morison por tramo activo con el agua media de sus extremos; la mitad a cada nudo (a uno dormido no le hace nada)
  drag_.assign(n, Eigen::Vector3d::Zero());
  for (int j : sleep_.active_links()) {
    const auto& l = topo_.links[static_cast<size_t>(j)];
    const auto& n0 = body_->m_nodes[l.a];
    const auto& n1 = body_->m_nodes[l.b];
    const Eigen::Vector3d f =
      core::morison_force<double>(eig(n0.m_x), eig(n1.m_x), eig(n0.m_v), eig(n1.m_v), 0.5 * (water_[l.a] + water_[l.b]),
                          l.diameter, rho, cfg_.hydro.cn, cfg_.hydro.ct);
    drag_[l.a] += 0.5 * f;
    drag_[l.b] += 0.5 * f;
  }

  // Por nudo: arrastre limitado, peso menos empuje y contacto con el suelo, en un único addForce.
  // La gravedad la pone net_weight: Stonefish deja a cero la del mundo blando (lo comprueba BuildScenario).
  for (int k : active) {
    const auto& node = topo_.nodes[static_cast<size_t>(k)];
    if (node.fixed) {
      continue;
    }
    const auto& b = body_->m_nodes[k];
    const Eigen::Vector3d pos = eig(b.m_x), vel = eig(b.m_v);
    const Eigen::Vector3d f = core::limit_drag<double>(drag_[k], water_[k] - vel, node.inertial_mass, dt) +
                              core::net_weight(node.mass, node.volume, rho, g) +
                              core::floor_force(pos, vel, node.inertial_mass, cfg_.mooring.floor_z, dt,
                                                cfg_.solver.friction);
    body_->addForce(bt(f), k);
  }

  cost_.forces_s = seconds_since(start);
  cost_.awake_tiles = sleep_.awake_tiles();
  cost_.tiles = sleep_.tile_count();
  cost_.active_nodes = active.size();
}

void BulletSolver::end_step(double dt)
{
  const int iterations = cfg_.solver.iterations;
  // Los anclajes usan lo que preparó btSoftBody::solveConstraints en el subpaso anterior (la boya apenas gira en
  // un subpaso); el primero, lo de las condiciones iniciales.
  double bullet = 0.0, contact = 0.0;
  auto start = std::chrono::steady_clock::now();
  robot_contact_.begin_step(*body_, body_->m_sst.sdt, sleep_.active_nodes());
  contact += seconds_since(start);
  for (int it = 0; it < iterations; ++it) {
    const btScalar ti = it / static_cast<btScalar>(iterations);
    start = std::chrono::steady_clock::now();
    for (int k = 0; k < body_->m_cfg.m_psequence.size(); ++k) {
      btSoftBody::getSolver(body_->m_cfg.m_psequence[k])(body_, 1, ti);
    }
    bullet += seconds_since(start);
    start = std::chrono::steady_clock::now();
    robot_contact_.project(*body_);
    contact += seconds_since(start);
  }
  robot_contact_.end_step();
  cost_.bullet_s = bullet;
  cost_.contact_s = contact;
  const btScalar vc = body_->m_sst.isdt * (1 - body_->m_cfg.kDP);
  for (int k : sleep_.active_nodes()) {  // los dormidos ya tienen v = 0 y ninguna fuerza
    btSoftBody::Node& node = body_->m_nodes[k];
    node.m_v = (node.m_x - node.m_q) * vc;
    node.m_f = btVector3(0, 0, 0);
  }
  copy_positions();
  if (diagnostics_) {
    sample_diagnostics(dt);
  }
}

void BulletSolver::sample_diagnostics(double dt)
{
  std::vector<Eigen::Vector3d> eye_v;
  std::vector<double> w;
  for (size_t k = 0; k < buoys_.size(); ++k) {
    const btRigidBody* rb = buoys_[k]->getRigidBody();
    const btVector3 eye = body_->m_nodes[topo_.marks.buoys[k].node].m_x;
    eye_v.push_back(eig(rb->getVelocityInLocalPoint(eye - rb->getCenterOfMassPosition())));
    w.push_back(rb->getAngularVelocity().length());
  }
  diagnostics_->sample_buoys(eye_v, w);
  diagnostics_->add_bullet(cost_);
  diagnostics_->sample(dt, positions_, topo_, cfg_.mooring.floor_z);
}

void BulletSolver::copy_positions()
{
  positions_.resize(static_cast<size_t>(body_->m_nodes.size()));
  for (int k = 0; k < body_->m_nodes.size(); ++k) {
    positions_[static_cast<size_t>(k)] = eig(body_->m_nodes[k].m_x).cast<float>();
  }
}

}  // namespace stonefish_net
