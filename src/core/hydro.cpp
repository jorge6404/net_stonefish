#include "stonefish_net/core/hydro.hpp"

namespace stonefish_net::core
{

Eigen::Vector3d net_weight(double mass, double volume, double rho, double g)
{
  return {0.0, 0.0, (mass - rho * volume) * g};
}

}  // namespace stonefish_net::core
