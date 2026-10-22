// ROS2SimulationManager con la red: tras el parser normal (tanque, ROV, ArUcos) añade la red, sus boyas,
// los bloques de anclaje y las fuerzas. Recibe la configuración, la topología y el asset de la boya ya
// cargados y validados (los carga main() antes de abrir la ventana).
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <stonefish_ros2/ROS2SimulationManager.h>

#include "stonefish_net/NetSolver.h"
#include "stonefish_net/SolidContacts.h"
#include "stonefish_net/core/buoy_asset.hpp"
#include "stonefish_net/core/net_config.hpp"
#include "stonefish_net/core/net_topology.hpp"
#include "stonefish_net/core/realtime.hpp"

class btCollisionObject;

namespace stonefish_net
{

class NetEntity;
class NetForces;

class NetSimulationManager : public sf::ROS2SimulationManager
{
public:
  NetSimulationManager(sf::Scalar rate, std::string scenarioPath, core::NetConfig cfg, core::NetTopology topo,
                       core::BuoyAsset buoy, const std::shared_ptr<rclcpp::Node>& nh);

  void BuildScenario() override;

  // El reloj de ROS con la recuperación acotada (core::CatchUpClock): si la física no llega a tiempo real va a
  // cámara lenta en vez de a saltos. Solo lo lee AdvanceSimulation, en el hilo de la física, y cada lectura alimenta
  // el informe de tiempo real (core::RealtimeReport).
  uint64_t getSimulationClock() const override;
  void SimulationClockSleep(uint64_t us) override;

private:
  // Post-tick de Bullet: el de Stonefish y después el de la red (NetForces::end_step).
  static void post_tick(btDynamicsWorld* world, btScalar dt);

  void warn_if_soft_gravity();
  void add_net_materials();
  std::unique_ptr<NetSolver> make_gpu_solver();
  std::unique_ptr<NetSolver> make_bullet_solver(NetEntity& net);
  std::vector<const btCollisionObject*> concave_statics();
  SolidContacts make_solid_contacts();
  void spawn_buoys();
  void spawn_anchors();
  void log_summary() const;
  // Con diagnostics, cada ventana: factor de tiempo real y coste del tick (la red y el resto); sin él, un aviso cada
  // 10 s si se está perdiendo tiempo
  void print_realtime(const core::RealtimeReport::Window& w, uint64_t wall_us) const;

  core::NetConfig cfg_;
  core::NetTopology topo_;
  core::BuoyAsset buoy_;
  std::vector<sf::SolidEntity*> buoys_;  // en el orden de topo_.marks.buoys
  SolidContacts solids_;
  std::unique_ptr<NetSolver> solver_;
  NetForces* net_forces_ = nullptr;  // del SimulationManager (AddActuator)

  // Del hilo de la física; mutables porque el reloj se lee en un getter const de Stonefish
  mutable core::CatchUpClock clock_;
  mutable core::RealtimeReport report_;
  mutable uint64_t last_warning_ = 0;
};

}  // namespace stonefish_net
