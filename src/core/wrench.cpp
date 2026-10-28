#include "stonefish_net/core/wrench.hpp"

namespace stonefish_net::core
{

Wrench wrench_in_frame(const std::vector<Wrench>& about_cg, const std::vector<Eigen::Vector3d>& cg,
                       const Eigen::Isometry3d& frame)
{
  // Primero en el mundo, con cada par llevado al origen del marco; luego a sus ejes
  const Eigen::Vector3d origin = frame.translation();
  Wrench world;
  for (size_t i = 0; i < about_cg.size() && i < cg.size(); ++i) {
    world.force += about_cg[i].force;
    world.torque += about_cg[i].torque + (cg[i] - origin).cross(about_cg[i].force);
  }
  const Eigen::Matrix3d to_frame = frame.linear().transpose();
  return {to_frame * world.force, to_frame * world.torque};
}

}  // namespace stonefish_net::core
