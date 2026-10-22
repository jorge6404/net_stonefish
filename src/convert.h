// Conversiones entre Eigen (net_core) y Bullet (sf::Vector3 es btVector3).
#pragma once

#include <Eigen/Core>
#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btVector3.h>

namespace stonefish_net
{

inline Eigen::Vector3d eig(const btVector3& v) { return {v.getX(), v.getY(), v.getZ()}; }
inline btVector3 bt(const Eigen::Vector3d& v) { return btVector3(v.x(), v.y(), v.z()); }

inline Eigen::Matrix3d eig(const btMatrix3x3& m)
{
  Eigen::Matrix3d res;
  res << m[0][0], m[0][1], m[0][2],
         m[1][0], m[1][1], m[1][2],
         m[2][0], m[2][1], m[2][2];
  return res;
}

}  // namespace stonefish_net
