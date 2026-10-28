// Fuerza y par que la red hace sobre los sólidos, y su suma expresada en el marco de uno de ellos (lo que mediría un
// sensor de fuerza montado ahí). Sin ROS ni Stonefish.
#pragma once

#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace stonefish_net::core
{

// Fuerza (N) y par (N·m) en el mundo (NED): el par, respecto al centro de gravedad del sólido que la recibe
struct Wrench
{
  Eigen::Vector3d force = Eigen::Vector3d::Zero();
  Eigen::Vector3d torque = Eigen::Vector3d::Zero();
};

// Suma de las fuerzas about_cg[i], cada una con su par respecto a cg[i] (mundo), expresada en `frame` (marco →
// mundo): la fuerza en sus ejes y el par respecto a su origen, también en sus ejes.
Wrench wrench_in_frame(const std::vector<Wrench>& about_cg, const std::vector<Eigen::Vector3d>& cg,
                       const Eigen::Isometry3d& frame);

}  // namespace stonefish_net::core
