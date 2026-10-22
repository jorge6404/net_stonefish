// Hidrodinámica de la red: Morison por tramo y peso menos empuje por nudo (NED: z abajo).
// Morison y su límite son plantillas en el escalar: double en BulletSolver, float en la referencia XPBD (core/xpbd_ref),
// que hace las mismas cuentas que los shaders. Mismas operaciones en el mismo orden en los dos.
#pragma once

#include <cmath>

#include <Eigen/Core>

namespace stonefish_net::core
{

template <typename S>
using Vec3 = Eigen::Matrix<S, 3, 1>;

// Fuerza total sobre un tramo cilíndrico p0-p1 (el llamador da la mitad a cada nudo).
// u = water − (v0 + v1)/2, separada en normal y tangencial al tramo:
// F = ½ ρ cn d L |u_n| u_n + ½ ρ ct π d L |u_t| u_t. Tramo degenerado → 0.
template <typename S>
Vec3<S> morison_force(const Vec3<S>& p0, const Vec3<S>& p1, const Vec3<S>& v0, const Vec3<S>& v1, const Vec3<S>& water,
                      S diameter, S rho, S cn, S ct)
{
  const Vec3<S> axis = p1 - p0;
  const S length = axis.norm();
  if (length < S(1e-9)) {
    return Vec3<S>::Zero();
  }
  const Vec3<S> t = axis / length;
  const Vec3<S> u = water - S(0.5) * (v0 + v1);
  const Vec3<S> ut = u.dot(t) * t;
  const Vec3<S> un = u - ut;
  return S(0.5) * rho * diameter * length * (cn * un.norm() * un + ct * static_cast<S>(M_PI) * ut.norm() * ut);
}

// Arrastre de un nudo limitado para que en un paso dt no cambie su velocidad relativa al agua
// (u_rel = agua − nudo) más que hasta anularla: |F| <= m·|u_rel|/dt. Sin esto el arrastre cuadrático
// explícito sobre nudos de ~1 g diverge en cuanto la velocidad relativa supera unos m/s.
template <typename S>
Vec3<S> limit_drag(const Vec3<S>& drag, const Vec3<S>& u_rel, S inertial_mass, S dt)
{
  const S max = inertial_mass * u_rel.norm() / dt;
  const S norm = drag.norm();
  if (max <= S(0)) {
    return Vec3<S>::Zero();
  }
  return norm > max ? Vec3<S>(drag * (max / norm)) : drag;
}

// (0, 0, (mass − ρ·volume)·g)
Eigen::Vector3d net_weight(double mass, double volume, double rho, double g);

}  // namespace stonefish_net::core
