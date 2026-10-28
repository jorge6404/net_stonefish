// Hidrodinámica de la red: Morison por tramo y peso menos empuje por nudo (NED: z abajo).
// Morison y su límite son plantillas en el escalar: double en BulletSolver, float en la referencia XPBD (core/xpbd_ref),
// que hace las mismas cuentas que los shaders. Mismas operaciones en el mismo orden en los dos.
#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Core>

namespace stonefish_net::core
{

template <typename S>
using Vec3 = Eigen::Matrix<S, 3, 1>;

// Cn de un cilindro en flujo normal (White, Viscous Fluid Flow: 1 + 10 Re^(−2/3), para 1 ≤ Re ≤ 2·10⁵), con Re
// acotado a ≥ 1: con el agua quieta respecto al hilo la fuerza tiende a 0 sin dividir por 0
template <typename S>
S cylinder_cn(S re)
{
  const S r = std::max(re, S(1));
  return S(1) + S(10) / std::pow(r * r, S(1) / S(3));
}

// Cd de una esfera (White: 24/Re + 6/(1 + √Re) + 0,4, para Re ≤ 2·10⁵), con Re acotado a ≥ 0,01
template <typename S>
S sphere_cd(S re)
{
  const S r = std::max(re, S(0.01));
  return S(24) / r + S(6) / (S(1) + std::sqrt(r)) + S(0.4);
}

// Cd del nudo sin viscosidad (esfera a Re ~ 10³)
inline constexpr double kKnotCd = 0.47;

// Fuerza total sobre un tramo cilíndrico p0-p1 (el llamador da la mitad a cada nudo).
// u = water − (v0 + v1)/2, separada en normal y tangencial al tramo:
// F = ½ ρ Cn d L |u_n| u_n + ½ ρ ct π d L |u_t| u_t, con Cn = cylinder_cn(|u_n| d/ν) si nu > 0 y cn si no.
// Tramo degenerado → 0.
template <typename S>
Vec3<S> morison_force(const Vec3<S>& p0, const Vec3<S>& p1, const Vec3<S>& v0, const Vec3<S>& v1, const Vec3<S>& water,
                      S diameter, S rho, S cn, S ct, S nu)
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
  const S cn_re = nu > S(0) ? cylinder_cn(un.norm() * diameter / nu) : cn;
  return S(0.5) * rho * diameter * length * (cn_re * un.norm() * un + ct * static_cast<S>(M_PI) * ut.norm() * ut);
}

// Arrastre de la esfera de un nudo de diámetro dk que va a v en agua a `water`: ½ ρ Cd (π/4) dk² |u| u, u = water − v,
// con Cd = sphere_cd(|u| dk/ν) si nu > 0 y kKnotCd si no. dk <= 0 → 0.
template <typename S>
Vec3<S> knot_drag(const Vec3<S>& v, const Vec3<S>& water, S dk, S rho, S nu)
{
  if (dk <= S(0)) {
    return Vec3<S>::Zero();
  }
  const Vec3<S> u = water - v;
  const S speed = u.norm();
  const S cd = nu > S(0) ? sphere_cd(speed * dk / nu) : static_cast<S>(kKnotCd);
  return S(0.5) * rho * cd * static_cast<S>(M_PI) / S(4) * dk * dk * speed * u;
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
