#include "stonefish_net/core/xpbd.hpp"

#include <cmath>

namespace stonefish_net::core
{

Eigen::Isometry3d interpolate_pose(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b, double s)
{
  const Eigen::Quaterniond qa(a.linear()), qb(b.linear());
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.linear() = qa.slerp(s, qb).toRotationMatrix();
  out.translation() = (1.0 - s) * a.translation() + s * b.translation();
  return out;
}

Eigen::Vector3d reaction_force(const Eigen::Vector3d& moved, double dt, int substeps)
{
  return -moved * static_cast<double>(substeps) / (dt * dt);
}

Eigen::Matrix3f point_inv_mass(double mass, const Eigen::Matrix3d& inv_inertia, const Eigen::Vector3d& r)
{
  Eigen::Matrix3d rx;
  rx << 0.0, -r.z(), r.y(), r.z(), 0.0, -r.x(), -r.y(), r.x(), 0.0;  // rx · f = r × f
  return (Eigen::Matrix3d::Identity() / mass - rx * inv_inertia * rx).cast<float>();
}

Eigen::Vector3d eye_impulse(const Eigen::Matrix3d& inv_mass, const Eigen::Vector3d& dv)
{
  return inv_mass.ldlt().solve(dv);
}

float substep_damping(double damping, int substeps)
{
  return static_cast<float>(1.0 - std::pow(1.0 - damping, 1.0 / substeps));
}

}  // namespace stonefish_net::core
