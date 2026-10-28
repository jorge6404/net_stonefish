#include "stonefish_net/NetSimulationManager.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <set>
#include <vector>

#include <SDL2/SDL.h>
#include <Stonefish/core/MaterialManager.h>
#include <Stonefish/core/SimulationApp.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <Stonefish/actuators/Thruster.h>
#include <Stonefish/core/Robot.h>
#include <Stonefish/entities/SolidEntity.h>
#include <Stonefish/entities/StaticEntity.h>
#include <Stonefish/entities/solids/Polyhedron.h>
#include <Stonefish/entities/statics/Obstacle.h>

#include "convert.h"
#include "stonefish_net/BulletSolver.h"
#include "stonefish_net/NetEntity.h"
#include "stonefish_net/GpuSolver.h"
#include "stonefish_net/NetForces.h"

namespace stonefish_net
{
namespace
{
constexpr double kAnchorBlockSide = 0.30;  // bloque de hormigón del fondeo (alto: mooring.block_height)

// Materiales de los sólidos de la red (boyas y bloques de anclaje)
struct NetMaterial
{
  const char* name;
  double density, restitution;
};
constexpr NetMaterial kNetMaterials[] = {{"Polyethylene", 950.0, 0.3}, {"Concrete", 2400.0, 0.2}};

// El enlace <robot>/<link> del escenario, o nullptr
sf::SolidEntity* robot_link(sf::SimulationManager& sm, const std::string& robot, const std::string& link)
{
  sf::Robot* r = sm.getRobot(robot);
  return r != nullptr ? r->getLink(robot + "/" + link) : nullptr;
}

// Los dedos de una pinza del YAML como cuerpos de contacto (robot_contact o robot_contact_auto por su enlace
// <robot>/<dedo>), en su orden, hasta el primero que no tiene contacto propio (que queda en `missing`)
std::vector<size_t> finger_bodies(const core::NetConfig::Gripper& gc, const SolidContacts& solids,
                                  sf::SimulationManager& sm, std::string& missing)
{
  const auto& bodies = solids.bodies();
  std::vector<size_t> out;
  for (const auto& finger : gc.fingers) {
    const sf::SolidEntity* link = robot_link(sm, gc.robot, finger);
    const auto it = std::find_if(bodies.begin(), bodies.end(), [link](const auto& b) { return b.solid == link; });
    if (link == nullptr || it == bodies.end()) {
      missing = finger;
      break;
    }
    out.push_back(static_cast<size_t>(it - bodies.begin()));
  }
  return out;
}

// Las pinzas del YAML con sus dedos como cuerpos de contacto; una pinza con algún dedo sin contacto propio se ignora
// con un aviso
std::vector<GpuSolver::Gripper> resolve_grippers(const std::vector<core::NetConfig::Gripper>& config,
                                                 const SolidContacts& solids, sf::SimulationManager& sm,
                                                 const rclcpp::Logger& logger)
{
  std::vector<GpuSolver::Gripper> grippers;
  for (const auto& gc : config) {
    GpuSolver::Gripper g{gc.robot + "/" + gc.fingers[0], {gc.grip_margin, gc.release_gap, {}, gc.max_pull}};
    for (size_t f = 1; f < gc.fingers.size(); ++f) g.name += "+" + gc.fingers[f];
    std::string missing;
    g.spec.fingers = finger_bodies(gc, solids, sm, missing);
    if (g.spec.fingers.size() != gc.fingers.size()) {
      RCLCPP_WARN(logger, "Red: la pinza %s se ignora: el dedo '%s' no existe o no tiene contacto propio "
                  "(robot_contact o robot_contact_auto)", g.name.c_str(), missing.c_str());
      continue;
    }
    for (size_t f = 0; f < g.spec.fingers.size(); ++f) {
      if (g.spec.fingers[f] >= core::kMaxExcludingBodies) {
        RCLCPP_WARN(logger, "Red: el dedo '%s' de la pinza %s es el cuerpo de contacto %zu (solo los %zu primeros "
                    "pueden ignorar nudos): chocará con lo que agarre", gc.fingers[f].c_str(), g.name.c_str(),
                    g.spec.fingers[f], core::kMaxExcludingBodies);
      }
    }
    RCLCPP_INFO(logger, "Red: pinza %s (grip_margin %.3f m, release_gap %.3f m)", g.name.c_str(), gc.grip_margin,
                gc.release_gap);
    grippers.push_back(std::move(g));
  }
  return grippers;
}

void log_box(const rclcpp::Logger& logger, const char* what, const std::string& name, const core::RoundedBox& box)
{
  RCLCPP_INFO(logger, "Red: %s con '%s': semiejes (%.3f, %.3f, %.3f) m, centro (%.3f, %.3f, %.3f) m, redondeo %.3f m",
              what, name.c_str(), box.half_extents.x(), box.half_extents.y(), box.half_extents.z(),
              box.center_offset.x(), box.center_offset.y(), box.center_offset.z(), box.rounding);
}
}  // namespace

NetSimulationManager::NetSimulationManager(sf::Scalar rate, std::string scenarioPath, core::NetConfig cfg,
                                           core::NetTopology topo, core::BuoyAsset buoy,
                                           const std::shared_ptr<rclcpp::Node>& nh)
: sf::ROS2SimulationManager(rate, std::move(scenarioPath), nh),
  cfg_(std::move(cfg)),
  topo_(std::move(topo)),
  buoy_(std::move(buoy)),
  clock_(static_cast<uint64_t>(core::kCatchUpTicks * 1e6 / rate))
{
}

uint64_t NetSimulationManager::getSimulationClock() const
{
  const uint64_t wall = sf::ROS2SimulationManager::getSimulationClock();
  const uint64_t clock = clock_.read(wall);
  const auto window = report_.read(wall, getSimulationTime(), clock_.dropped_us());
  if (window && net_forces_ != nullptr) {
    print_realtime(*window, wall);
  }
  return clock;
}

void NetSimulationManager::SimulationClockSleep(uint64_t us)
{
  const auto start = std::chrono::steady_clock::now();
  sf::ROS2SimulationManager::SimulationClockSleep(us);
  report_.add_sleep(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count());
}

void NetSimulationManager::print_realtime(const core::RealtimeReport::Window& w, uint64_t wall_us) const
{
  const double tick_ms = 1e3 / getStepsPerSecond();
  const double factor = w.sim_s / w.wall_s;
  if (cfg_.diagnostics.enabled) {
    std::fprintf(stderr,
                 "[stonefish_net] tiempo real: factor %.2f (%.2f s simulados en %.2f s de reloj, %ld ticks), tick "
                 "%.2f ms de %.2f ms: red %.2f ms (máx %.2f), el resto %.2f ms; perdidos %.0f ms por no llegar\n",
                 factor, w.sim_s, w.wall_s, w.ticks, w.busy_ms, tick_ms, w.net_ms, w.net_max_ms,
                 std::max(w.busy_ms - w.net_ms, 0.0), w.dropped_ms);
  } else if (w.dropped_ms > 0.05 * 1e3 * w.wall_s && wall_us >= std::max(last_warning_, w.started_us) + 10'000'000) {
    std::fprintf(stderr,
                 "[stonefish_net] AVISO: la física no llega a tiempo real (factor %.2f, tick %.2f ms de %.2f ms, la red "
                 "%.2f ms): va a cámara lenta; baja solver.substeps o simulation_rate\n",
                 factor, w.busy_ms, tick_ms, w.net_ms);
    last_warning_ = wall_us;
  }
}

void NetSimulationManager::BuildScenario()
{
  sf::ROS2SimulationManager::BuildScenario();
  warn_if_soft_gravity();
  add_net_materials();
  auto* net = new NetEntity("net", topo_, cfg_);
  AddEntity(net);
  spawn_buoys();
  spawn_anchors();
  solids_ = make_solid_contacts();
  solver_ = cfg_.solver.backend == core::NetConfig::Backend::Gpu ? make_gpu_solver() : make_bullet_solver(*net);
  net->set_solver(solver_.get());
  net_forces_ = new NetForces("net_forces", solver_.get(), this, &NetSimulationManager::post_tick, report_);
  AddActuator(net_forces_);
  make_wrenches();
  log_summary();
}

void NetSimulationManager::make_wrenches()
{
  const std::string prefix = "/stonefish_net/wrench/";
  std::vector<NetWrenches::Group> groups;
  for (unsigned int r = 0; sf::Robot* robot = getRobot(r); ++r) {
    NetWrenches::Group g{prefix + robot->getName(), robot->getName() + "/base_link", robot->getBaseLink(), {}};
    for (size_t b = 0; b < solids_.bodies().size(); ++b) {
      if (solids_.bodies()[b].robot == robot) g.bodies.push_back(b);
    }
    if (!g.bodies.empty() && g.frame != nullptr) groups.push_back(std::move(g));
  }
  // Las pinzas solo agarran con gpu; con bullet no tienen topic propio
  for (size_t k = 0; k < cfg_.grippers.size() && cfg_.solver.backend == core::NetConfig::Backend::Gpu; ++k) {
    const auto& gc = cfg_.grippers[k];
    std::string missing;
    std::vector<size_t> fingers = finger_bodies(gc, solids_, *this, missing);
    if (fingers.size() != gc.fingers.size()) continue;
    sf::SolidEntity* first = solids_.bodies()[fingers.front()].solid;
    groups.push_back({prefix + gc.robot + "/gripper_" + std::to_string(k), first->getName(), first, std::move(fingers)});
  }
  wrenches_ = std::make_unique<NetWrenches>(*nh_, std::move(groups));
}

// La red en la GPU. Necesita la ventana de SDL de Stonefish: aquí, en el hilo principal, está activo su contexto de
// dibujo y de él sale la ventana. El contexto de cálculo se crea en el primer tick, ya en el hilo de la física.
std::unique_ptr<NetSolver> NetSimulationManager::make_gpu_solver()
{
  auto fatal = [&](const char* why) {
    RCLCPP_FATAL(nh_->get_logger(), "Red: solver.backend gpu %s; usa solver.backend: bullet", why);
    std::exit(1);
  };
  if (!sf::SimulationApp::getApp()->hasGraphics()) {
    fatal("necesita la ventana del simulador (modo gráfico)");
  }
  SDL_Window* window = SDL_GL_GetCurrentWindow();
  if (window == nullptr) {
    fatal("no encuentra la ventana de SDL del simulador");
  }
  if (cfg_.sleep.enabled) {
    RCLCPP_WARN(nh_->get_logger(), "Red: sleep se ignora con solver.backend gpu (toda la red se calcula en la GPU)");
  }
  return std::make_unique<GpuSolver>(topo_, cfg_, this, solids_, buoys_, buoy_.eye, window,
                                     ament_index_cpp::get_package_share_directory("stonefish_net") + "/shaders",
                                     resolve_grippers(cfg_.grippers, solids_, *this, nh_->get_logger()), thrusters());
}

// La red como cuerpo blando de Bullet, con el contacto propio en la CPU (RobotContact)
std::unique_ptr<NetSolver> NetSimulationManager::make_bullet_solver(NetEntity& net)
{
  if (!cfg_.grippers.empty()) {
    RCLCPP_WARN(nh_->get_logger(), "Red: grippers se ignora con solver.backend bullet (las pinzas solo agarran con gpu)");
  }
  return std::make_unique<BulletSolver>(topo_, cfg_, this, &net, solids_, buoys_, concave_statics(), thrusters());
}

// Los propulsores de todos los robots, cuyo chorro empuja la red; ninguno con thruster_jets.enabled false
std::vector<sf::Thruster*> NetSimulationManager::thrusters()
{
  std::vector<sf::Thruster*> out;
  if (!cfg_.thruster_jets.enabled) return out;
  for (unsigned int r = 0; sf::Robot* robot = getRobot(r); ++r) {
    for (size_t a = 0; sf::Actuator* act = robot->getActuator(a); ++a) {
      if (act->getType() == sf::ActuatorType::THRUSTER) out.push_back(static_cast<sf::Thruster*>(act));
    }
  }
  RCLCPP_INFO(nh_->get_logger(), "Red: chorro de %zu propulsores sobre la red", out.size());
  if (out.size() > core::kMaxJets) {
    RCLCPP_WARN(nh_->get_logger(), "Red: solo los %zu primeros propulsores empujan la red con su chorro", core::kMaxJets);
  }
  return out;
}

void NetSimulationManager::post_tick(btDynamicsWorld* world, btScalar dt)
{
  sf::SimulationManager::SimulationPostTickCallback(world, dt);
  auto* self = static_cast<NetSimulationManager*>(world->getWorldUserInfo());
  self->net_forces_->end_step(dt);
  self->wrenches_->publish(self->solids_, self->nh_->get_clock()->now());
}

// BulletSolver aplica el peso con net_weight; si Stonefish activara la gravedad del mundo blando se contaría dos
// veces. Hoy la deja a cero (SimulationManager::InitializeSolver).
void NetSimulationManager::warn_if_soft_gravity()
{
  if (!getSoftBodyWorldInfo().m_gravity.fuzzyZero()) {
    RCLCPP_WARN(nh_->get_logger(), "La gravedad del mundo blando no es cero: la red pesará el doble");
  }
}

// Los materiales de la red se crean aquí y no en el .scn: Stonefish solo lee el primer <materials> y los
// <include> se pegan al final, así que un bloque en el .scn de la red taparía el del escenario base (y sus
// objetos caerían en silencio al material 0). Si el escenario ya define uno, se respeta el suyo: crearlo
// otra vez lo renombraría.
void NetSimulationManager::add_net_materials()
{
  sf::MaterialManager* mm = getMaterialManager();
  const std::vector<std::string> existing = mm->GetMaterialsList();
  for (const NetMaterial& m : kNetMaterials) {
    if (std::find(existing.begin(), existing.end(), m.name) == existing.end()) {
      mm->CreateMaterial(m.name, m.density, m.restitution);
    }
  }
}

// Mallas estáticas cóncavas (el tanque), que la red de Bullet no debe tocar; el suelo lo pone BulletSolver.
std::vector<const btCollisionObject*> NetSimulationManager::concave_statics()
{
  std::vector<const btCollisionObject*> out;
  for (unsigned int i = 0; sf::Entity* ent = getEntity(i); ++i) {
    if (ent->getType() != sf::EntityType::STATIC) {
      continue;
    }
    btRigidBody* rb = static_cast<sf::StaticEntity*>(ent)->getRigidBody();
    if (rb != nullptr && rb->getCollisionShape()->isConcave()) {
      out.push_back(rb);
      RCLCPP_INFO(nh_->get_logger(), "Red: sin colisión con la malla cóncava '%s'", ent->getName().c_str());
    }
  }
  return out;
}

// Los cuerpos que tocan la red con contacto propio (SolidContacts). Primero los enlaces de robot_contact, con su
// caja medida; luego, con robot_contact_auto, el resto de enlaces de todos los robots con su caja envolvente; y, con
// solid_contact_auto, los sólidos dinámicos que no son robots (boyas incluidas: ya están creadas), también con su
// caja envolvente. El orinque de cada boya no toca su boya: el anclaje lo lleva a su ojal.
SolidContacts NetSimulationManager::make_solid_contacts()
{
  std::vector<SolidContacts::Body> bodies;
  std::set<const sf::SolidEntity*> taken, robot_links;
  for (unsigned int r = 0; sf::Robot* robot = getRobot(r); ++r) {
    for (size_t k = 0; sf::SolidEntity* link = robot->getLink(k); ++k) robot_links.insert(link);
  }

  for (const auto& rl : cfg_.robot_contact) {
    sf::SolidEntity* link = robot_link(*this, rl.robot, rl.link);
    if (link == nullptr) {
      RCLCPP_WARN(nh_->get_logger(), "robot_contact: no existe el enlace '%s' del robot '%s' en el escenario; se ignora",
                  rl.link.c_str(), rl.robot.c_str());
      continue;
    }
    bodies.push_back({getRobot(rl.robot), link, rl.box, rl.friction, {}});
    taken.insert(link);
    RCLCPP_INFO(nh_->get_logger(), "Red: contacto propio con '%s'", link->getName().c_str());
  }

  // Caja envolvente de un sólido; nada (con aviso) si no tiene forma de colisión
  auto fit = [&](sf::SolidEntity& solid, double rounding_fraction) {
    const auto box = SolidContacts::fit_box(solid, rounding_fraction);
    if (!box) {
      RCLCPP_WARN(nh_->get_logger(), "Red: '%s' no tiene forma de colisión; sin contacto propio", solid.getName().c_str());
    }
    return box;
  };

  const auto& robots = cfg_.robot_contact_auto;
  if (robots.enabled) {
    for (unsigned int r = 0; sf::Robot* robot = getRobot(r); ++r) {
      for (size_t k = 0; sf::SolidEntity* link = robot->getLink(k); ++k) {
        if (taken.count(link)) continue;
        if (const auto box = fit(*link, robots.rounding_fraction)) {
          bodies.push_back({robot, link, *box, robots.friction, {}});
          taken.insert(link);
          log_box(nh_->get_logger(), "contacto automático", link->getName(), *box);
        }
      }
    }
  }

  const auto& solids = cfg_.solid_contact_auto;
  if (solids.enabled) {
    for (unsigned int i = 0; sf::Entity* ent = getEntity(i); ++i) {
      if (ent->getType() != sf::EntityType::SOLID) continue;
      auto* solid = static_cast<sf::SolidEntity*>(ent);
      if (solid->getPhysicsMode() == sf::PhysicsMode::DISABLED || robot_links.count(solid) || taken.count(solid)) continue;
      if (const auto box = fit(*solid, solids.rounding_fraction)) {
        std::vector<int> excluded;
        const auto buoy = std::find(buoys_.begin(), buoys_.end(), solid);
        if (buoy != buoys_.end()) {
          excluded = core::dropper_nodes(topo_, topo_.marks.buoys[static_cast<size_t>(buoy - buoys_.begin())].node);
        }
        bodies.push_back({nullptr, solid, *box, solids.friction, std::move(excluded)});
        taken.insert(solid);
        log_box(nh_->get_logger(), "contacto automático (sólido)", solid->getName(), *box);
      }
    }
  }
  // Solo los primeros core::kMaxExcludingBodies cuerpos pueden ignorar nudos (el orinque de una boya)
  for (size_t b : core::cannot_exclude(bodies)) {
    RCLCPP_WARN(nh_->get_logger(), "Red: '%s' es el cuerpo de contacto %zu (solo los %zu primeros pueden ignorar nudos): "
                "chocará con los nudos que debía ignorar", bodies[b].solid->getName().c_str(), b, core::kMaxExcludingBodies);
  }
  return SolidContacts(std::move(bodies));
}

// Boyas con el ojal hacia abajo (girada 180° en x): así cuelga una boya real de un orinque tenso.
// Su look (net_buoy, con la textura del asset) está en el escenario.
void NetSimulationManager::spawn_buoys()
{
  buoys_.clear();
  const sf::Quaternion flip(sf::Vector3(1, 0, 0), M_PI);
  const sf::Vector3 eye_local = bt(buoy_.eye);
  for (size_t k = 0; k < topo_.marks.buoys.size(); ++k) {
    sf::PhysicsSettings phy;
    phy.mode = sf::PhysicsMode::FLOATING;
    phy.collisions = true;
    phy.buoyancy = true;
    auto* buoy = new sf::Polyhedron("net_buoy_" + std::to_string(k), phy, buoy_.visual_mesh, 1.0, sf::I4(),
                                    buoy_.physics_mesh, 1.0, sf::I4(), "Polyethylene", "net_buoy");
    buoy->ScalePhysicalPropertiesToArbitraryMass(cfg_.buoys.mass);  // hueca, no un sólido de polietileno
    const sf::Vector3 eye_world = bt(topo_.marks.buoys[k].eye);
    AddSolidEntity(buoy, sf::Transform(flip, eye_world - sf::Transform(flip, sf::V0()) * eye_local));
    buoys_.push_back(buoy);
  }
}

void NetSimulationManager::spawn_anchors()
{
  const double h = cfg_.mooring.block_height;
  for (size_t k = 0; k < topo_.marks.moorings.size(); ++k) {
    if (!topo_.marks.moorings[k].on_floor) continue;  // atada a la pared del tanque: sin bloque
    const sf::Vector3 base = bt(topo_.marks.moorings[k].block_base);
    auto* block = new sf::Obstacle("net_anchor_" + std::to_string(k), sf::Vector3(kAnchorBlockSide, kAnchorBlockSide, h),
                                   sf::I4(), "Concrete", "net_concrete");
    AddStaticEntity(block, sf::Transform(sf::IQ(), base - sf::Vector3(0, 0, h / 2)));
  }
}

void NetSimulationManager::log_summary() const
{
  RCLCPP_INFO(nh_->get_logger(), "Red: %zu nudos, %zu tramos, %zu boyas, %zu plomos", topo_.nodes.size(),
              topo_.links.size(), topo_.marks.buoys.size(), topo_.marks.leads.size());
  std::string tears;
  for (int s : topo_.damage.tear_sizes) tears += (tears.empty() ? "" : ", ") + std::to_string(s);
  RCLCPP_INFO(nh_->get_logger(), "Daño (semilla %d): %zu rotos [%s rombos], %d hilos sueltos cortados",
              cfg_.damage.seed, topo_.damage.tear_sizes.size(), tears.c_str(), topo_.damage.loose_cut);
}

}  // namespace stonefish_net
