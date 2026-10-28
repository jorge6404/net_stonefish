// La fuerza de la red en el marco de un sensor (core::wrench_in_frame), la que publica NetWrenches
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "stonefish_net/core/wrench.hpp"

namespace stonefish_net::test
{

namespace core = stonefish_net::core;

// Dos cuerpos con su fuerza y su par en el centro de gravedad, y un marco en (1, 0, 0) girado 90° en z. A mano, en el
// mundo y respecto a (1, 0, 0): fuerza (1, 0, 2) y par (0, −2, 0) + (0, 0, −1) + (0, 0, 0,5); en los ejes del marco,
// (v_y, −v_x, v_z)
TEST(Wrench, InFrame)
{
  Eigen::Isometry3d frame = Eigen::Isometry3d::Identity();
  frame.linear() = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  frame.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);
  const std::vector<core::Wrench> about_cg{{Eigen::Vector3d(0.0, 0.0, 2.0), Eigen::Vector3d::Zero()},
                                           {Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.5)}};
  const std::vector<Eigen::Vector3d> cg{Eigen::Vector3d(2.0, 0.0, 0.0), Eigen::Vector3d(1.0, 1.0, 0.0)};
  const core::Wrench got = core::wrench_in_frame(about_cg, cg, frame);
  EXPECT_LE((got.force - Eigen::Vector3d(0.0, -1.0, 2.0)).norm(), 1e-12);
  EXPECT_LE((got.torque - Eigen::Vector3d(-2.0, 0.0, -0.5)).norm(), 1e-12);
}

// Sin cuerpos, cero
TEST(Wrench, Empty)
{
  const core::Wrench got = core::wrench_in_frame({}, {}, Eigen::Isometry3d::Identity());
  EXPECT_TRUE(got.force.isZero() && got.torque.isZero());
}

}  // namespace stonefish_net::test
