// Hidrodinámica de la red (core/hydro.hpp): Cn(Re) del hilo, Cd(Re) del nudo y sus fuerzas
#include <gtest/gtest.h>

#include <cmath>

#include "stonefish_net/core/hydro.hpp"

namespace stonefish_net::test
{

namespace core = stonefish_net::core;
using V = Eigen::Vector3d;

// White: 1 + 10 Re^(−2/3)
TEST(Hydro, WhiteCylinder)
{
  EXPECT_NEAR(core::cylinder_cn(500.0), 1.159, 1e-3);
  EXPECT_NEAR(core::cylinder_cn(50.0), 1.737, 1e-3);
  EXPECT_EQ(core::cylinder_cn(0.0), core::cylinder_cn(1.0)) << "Re no se acota a 1";
}

// White: 24/Re + 6/(1 + √Re) + 0,4
TEST(Hydro, WhiteSphere)
{
  EXPECT_NEAR(core::sphere_cd(1000.0), 0.608, 1e-3);
  EXPECT_EQ(core::sphere_cd(0.0), core::sphere_cd(0.01)) << "Re no se acota a 0,01";
}

// Con el agua quieta respecto al hilo y al nudo, y viscosidad, la fuerza es 0 y finita (Re = 0)
TEST(Hydro, ReynoldsAtRest)
{
  const V f = core::morison_force<double>(V(0, 0, 0), V(0, 0.1, 0), V(0.2, 0, 0), V(0.2, 0, 0), V(0.2, 0, 0), 0.0025,
                                          1000.0, 1.2, 0.01, 1e-6);
  EXPECT_TRUE(f.allFinite() && f.isZero());
  const V k = core::knot_drag<double>(V(0.2, 0, 0), V(0.2, 0, 0), 0.006, 1000.0, 1e-6);
  EXPECT_TRUE(k.allFinite() && k.isZero());
}

// Sin viscosidad, el Cn constante de siempre: ½ ρ cn d L u²; con ella, el de White a su Re
TEST(Hydro, MorisonCn)
{
  const double d = 0.0025, L = 0.1, u = 0.2, rho = 1000.0;
  const V p0(0, 0, 0), p1(0, L, 0), still(0, 0, 0), water(u, 0, 0);
  const V constant = core::morison_force<double>(p0, p1, still, still, water, d, rho, 1.2, 0.01, 0.0);
  EXPECT_NEAR(constant.x(), 0.5 * rho * 1.2 * d * L * u * u, 1e-12);
  const V white = core::morison_force<double>(p0, p1, still, still, water, d, rho, 1.2, 0.01, 1e-6);
  EXPECT_NEAR(white.x(), 0.5 * rho * core::cylinder_cn(u * d / 1e-6) * d * L * u * u, 1e-12);
}

// El nudo: ½ ρ Cd (π/4) dk² u², con Cd 0,47 sin viscosidad y el de White con ella; sin diámetro, nada
TEST(Hydro, KnotDrag)
{
  const double dk = 0.006, u = 0.2, rho = 1000.0;
  const double area = M_PI / 4.0 * dk * dk;
  EXPECT_NEAR(core::knot_drag<double>(V(0, 0, 0), V(u, 0, 0), dk, rho, 0.0).x(), 0.5 * rho * 0.47 * area * u * u, 1e-12);
  EXPECT_NEAR(core::knot_drag<double>(V(0, 0, 0), V(u, 0, 0), dk, rho, 1e-6).x(),
              0.5 * rho * core::sphere_cd(u * dk / 1e-6) * area * u * u, 1e-12);
  EXPECT_TRUE(core::knot_drag<double>(V(0, 0, 0), V(u, 0, 0), 0.0, rho, 1e-6).isZero());
}

}  // namespace stonefish_net::test
