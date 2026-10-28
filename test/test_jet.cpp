// El chorro de un propulsor sobre la red (core/jet.hpp)
#include <gtest/gtest.h>

#include <cmath>

#include "stonefish_net/core/jet.hpp"

namespace stonefish_net::test
{

namespace core = stonefish_net::core;

namespace
{
// Hélice de 0,1 m en el origen que empuja 10 N en +x: el agua sale en −x
core::Jet jet10() { return *core::jet_from_thrust(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(), 10.0, 0.1, 1000.0); }
}  // namespace

// u₀ = √(2T/(ρA)), r₀ = D/(2√2), contra el empuje
TEST(Jet, FromThrust)
{
  const core::Jet j = jet10();
  EXPECT_NEAR(j.speed, std::sqrt(2.0 * 10.0 / (1000.0 * M_PI * 0.05 * 0.05)), 1e-5);
  EXPECT_NEAR(j.speed, 1.596, 1e-3);
  EXPECT_NEAR(j.radius, 0.03536, 1e-5);
  EXPECT_TRUE(j.dir.isApprox(-Eigen::Vector3f::UnitX()));
}

// En el eje: u₀ dentro del núcleo y u₀/2 al doble de su largo
TEST(Jet, CoreAndDecay)
{
  const core::Jet j = jet10();
  const float core_len = core::kJetCore * 2.f * j.radius;
  EXPECT_NEAR(core::jet_velocity(j, Eigen::Vector3f(-2.f * j.radius, 0.f, 0.f), 0.1f).x(), -j.speed, 1e-5);
  EXPECT_NEAR(core::jet_velocity(j, Eigen::Vector3f(-2.f * core_len, 0.f, 0.f), 0.1f).x(), -0.5f * j.speed, 1e-5);
}

// Perfil gaussiano: a una anchura b del eje, e⁻¹ de lo del eje
TEST(Jet, Gaussian)
{
  const core::Jet j = jet10();
  const float x = 2.f * core::kJetCore * 2.f * j.radius, b = 0.1f * x;
  const Eigen::Vector3f v = core::jet_velocity(j, Eigen::Vector3f(-x, b, 0.f), 0.1f);
  EXPECT_NEAR(v.x(), -0.5f * j.speed * std::exp(-1.f), 1e-5);
  EXPECT_NEAR(v.y(), 0.f, 1e-7) << "el chorro no va por su eje";
}

// Aguas arriba de la hélice y más allá de su largo, nada
TEST(Jet, UpstreamAndFar)
{
  const core::Jet j = jet10();
  EXPECT_TRUE(core::jet_velocity(j, Eigen::Vector3f(0.5f, 0.f, 0.f), 0.1f).isZero());
  EXPECT_TRUE(core::jet_velocity(j, Eigen::Vector3f(-core::kJetLength * 2.f * j.radius - 0.01f, 0.f, 0.f), 0.1f).isZero());
}

// Sin empuje no hay chorro; con empuje negativo, el agua va hacia +eje; nunca NaN
TEST(Jet, ZeroAndReverseThrust)
{
  EXPECT_FALSE(core::jet_from_thrust(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(), 0.0, 0.1, 1000.0));
  EXPECT_FALSE(core::jet_from_thrust(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(), 5.0, 0.0, 1000.0));
  EXPECT_FALSE(core::jet_from_thrust(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(), std::nan(""), 0.1, 1000.0));
  const auto back = core::jet_from_thrust(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(), -10.0, 0.1, 1000.0);
  ASSERT_TRUE(back);
  EXPECT_TRUE(back->dir.isApprox(Eigen::Vector3f::UnitX()));
  EXPECT_TRUE(core::jet_velocity(*back, Eigen::Vector3f(0.3f, 0.f, 0.f), 0.1f).allFinite());
}

// Varios chorros se suman
TEST(Jet, Sum)
{
  const core::Jet j = jet10();
  const Eigen::Vector3f p(-0.3f, 0.01f, 0.f);
  EXPECT_TRUE(core::jet_velocity({j, j}, p, 0.1f).isApprox(2.f * core::jet_velocity(j, p, 0.1f)));
}

}  // namespace stonefish_net::test
