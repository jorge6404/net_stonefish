// Enganche del cálculo de la red (NetSolver) en el tick de Stonefish.
// Es un Actuator porque Stonefish llama a Update(dt) de todos los actuadores en el pre-tick, justo después de
// borrar las fuerzas: es el gancho para fuerzas propias. end_step lo llama el post-tick de Bullet
// (NetSimulationManager::post_tick), tras integrar los sólidos y detectar colisiones.
#pragma once

#include <string>
#include <vector>

#include <BulletDynamics/Dynamics/btDynamicsWorld.h>
#include <Stonefish/actuators/Actuator.h>

#include "stonefish_net/NetSolver.h"
#include "stonefish_net/core/realtime.hpp"

namespace sf
{
class SimulationManager;
}

namespace stonefish_net
{

class NetForces : public sf::Actuator
{
public:
  // post_tick: la función que Bullet debe llamar en el post-tick; tiene que acabar llamando a end_step.
  // report: recibe lo que cuesta la red (begin_step + end_step) en cada tick.
  NetForces(std::string name, NetSolver* solver, sf::SimulationManager* sm, btInternalTickCallback post_tick,
            core::RealtimeReport& report);

  void Update(sf::Scalar dt) override;
  void end_step(sf::Scalar dt);
  sf::ActuatorType getType() const override { return sf::ActuatorType::PUSH; }
  std::vector<sf::Renderable> Render() override { return {}; }

private:
  NetSolver* solver_;
  sf::SimulationManager* sm_;
  btInternalTickCallback post_tick_;
  core::RealtimeReport& report_;
  double tick_ms_ = 0.0;  // del tick en curso
};

}  // namespace stonefish_net
