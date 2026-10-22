#include "stonefish_net/NetForces.h"

#include <chrono>
#include <utility>

#include <Stonefish/core/SimulationManager.h>

namespace stonefish_net
{

NetForces::NetForces(std::string name, NetSolver* solver, sf::SimulationManager* sm, btInternalTickCallback post_tick,
                     core::RealtimeReport& report)
: sf::Actuator(std::move(name)), solver_(solver), sm_(sm), post_tick_(post_tick), report_(report)
{
}

void NetForces::Update(sf::Scalar dt)
{
  // El post-tick se engancha en cada subpaso porque Stonefish reinstala sus propios callbacks al resolver las
  // condiciones iniciales (SolveICProblem), después de construir la escena.
  sm_->getDynamicsWorld()->setInternalTickCallback(post_tick_, sm_, false);
  const auto start = std::chrono::steady_clock::now();
  solver_->begin_step(dt);
  tick_ms_ = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

void NetForces::end_step(sf::Scalar dt)
{
  const auto start = std::chrono::steady_clock::now();
  solver_->end_step(dt);
  tick_ms_ += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  report_.add_net_tick(tick_ms_);
}

}  // namespace stonefish_net
