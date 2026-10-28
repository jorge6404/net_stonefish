// La red como cuerpo blando de Bullet (solver.backend: bullet).
//
// begin_step (pre-tick): Morison por tramo, peso − empuje y suelo por nudo, en un único addForce; la reacción
// pendiente sobre los sólidos (SolidContacts); las baldosas dormidas (NetSleep).
// end_step (post-tick, tras integrar y detectar colisiones y antes de que Bullet resuelva el cuerpo blando): las
// iteraciones de posiciones de Bullet (anclajes, contactos nativos, tramos) con el contacto con los sólidos al
// final de cada una, y las velocidades a partir de las posiciones. Las mismas que haría
// btSoftBody::solveConstraints, que queda sin iteraciones de posición (begin_step pone piterations a 0).
#pragma once

#include <memory>
#include <vector>

#include <Eigen/Core>

#include "stonefish_net/NetSleep.h"
#include "stonefish_net/NetSolver.h"
#include "stonefish_net/RobotContact.h"
#include "stonefish_net/SolidContacts.h"
#include "stonefish_net/core/net_config.hpp"
#include "stonefish_net/core/net_topology.hpp"
#include "stonefish_net/diagnostics.h"

class btSoftBody;
class btCollisionObject;

namespace sf
{
class Entity;
class SimulationManager;
class SolidEntity;
class Thruster;
}  // namespace sf

namespace stonefish_net
{

class BulletSolver : public NetSolver
{
public:
  // Crea el cuerpo blando y lo añade al mundo: desde ese momento es del mundo de Bullet
  // (SimulationManager::DestroyScenario borra todos los objetos de colisión antes que las entidades).
  // owner: la entidad de la red; Stonefish lee el user pointer de los objetos de colisión como sf::Entity*
  // (selección con el ratón, callbacks de contacto, océano).
  // solids: los cuerpos con contacto propio (RobotContact), que no chocan con la red por Bullet (SDF_RS solo mira los
  // nudos). buoys: una por topo.marks.buoys, en orden; el extremo superior de cada orinque se ancla a la suya.
  // ignored: objetos con los que la red no colisiona (mallas cóncavas como el tanque: el SDF de Bullet solo mide
  // formas convexas y con una malla da contactos falsos que lanzan los nudos).
  // thrusters: los propulsores cuyo chorro empuja la red (vacío con thruster_jets.enabled false).
  BulletSolver(const core::NetTopology& topo, const core::NetConfig& cfg, sf::SimulationManager* sm, sf::Entity* owner,
               SolidContacts& solids, const std::vector<sf::SolidEntity*>& buoys,
               const std::vector<const btCollisionObject*>& ignored, std::vector<sf::Thruster*> thrusters);

  void begin_step(double dt) override;
  void end_step(double dt) override;
  const std::vector<Eigen::Vector3f>& positions() const override { return positions_; }

private:
  void copy_positions();
  void sample_diagnostics(double dt);

  const core::NetTopology& topo_;
  const core::NetConfig& cfg_;
  sf::SimulationManager* sm_;
  SolidContacts& solids_;
  std::vector<sf::SolidEntity*> buoys_;
  std::vector<sf::Thruster*> thrusters_;
  btSoftBody* body_ = nullptr;
  RobotContact robot_contact_;
  NetSleep sleep_;  // qué nudos y tramos se calculan en cada subpaso
  std::unique_ptr<NetDiagnostics> diagnostics_;  // nullptr si diagnostics.enabled es false
  NetDiagnostics::BulletCost cost_;               // del tick en curso, para diagnostics_
  std::vector<Eigen::Vector3d> water_;  // velocidad del agua en cada nudo activo, muestreada una vez por subpaso
  std::vector<Eigen::Vector3d> drag_;   // arrastre de Morison acumulado por nudo
  std::vector<Eigen::Vector3f> positions_;
};

}  // namespace stonefish_net
