// Cálculo de la red en cada tick de Bullet, sea con el cuerpo blando de Bullet (BulletSolver) o en la GPU.
// NetForces lo engancha a Stonefish: begin_step en el pre-tick (Actuator::Update) y end_step en el post-tick,
// los dos en el hilo de la física. NetEntity dibuja positions().
#pragma once

#include <vector>

#include <Eigen/Core>

namespace stonefish_net
{

class NetSolver
{
public:
  virtual ~NetSolver() = default;

  virtual void begin_step(double dt) = 0;  // pre-tick, justo después de que Stonefish borre las fuerzas
  virtual void end_step(double dt) = 0;    // post-tick, después de que Bullet integre los sólidos

  // Posición de cada nudo de la topología, en el mundo (NED), al final del último tick.
  virtual const std::vector<Eigen::Vector3f>& positions() const = 0;
};

}  // namespace stonefish_net
