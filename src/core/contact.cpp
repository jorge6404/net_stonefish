#include "stonefish_net/core/contact.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace stonefish_net::core
{

Eigen::Vector3d floor_force(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, double mass, double floor_z,
                            double dt, double mu)
{
  const double pen = pos.z() - floor_z;
  if (pen <= 0.0) {
    return Eigen::Vector3d::Zero();
  }
  const double fz = std::min(0.0, mass * (-pen / dt - vel.z()) / dt);
  const Eigen::Vector3d vt(vel.x(), vel.y(), 0.0);
  const double speed = vt.norm();
  Eigen::Vector3d f(0.0, 0.0, fz);
  if (speed > 0.0) {
    f -= vt / speed * std::min(mu * -fz, mass * speed / dt);
  }
  return f;
}

namespace
{
// Distancia con signo a la caja redondeada de un punto en el marco de la caja (centrado en center_offset):
// distancia a la caja interior (sin redondeo) − rounding. Devuelve también q = |local| − inner.
double local_signed_distance(const RoundedBox& box, const Eigen::Vector3d& local, Eigen::Vector3d& q)
{
  const Eigen::Vector3d inner = (box.half_extents - Eigen::Vector3d::Constant(box.rounding)).cwiseMax(1e-4);
  q = local.cwiseAbs() - inner;
  return q.cwiseMax(0.0).norm() + std::min(q.maxCoeff(), 0.0) - box.rounding;
}
}  // namespace

double box_signed_distance(const RoundedBox& box, const Eigen::Isometry3d& pose, const Eigen::Vector3d& p)
{
  Eigen::Vector3d q;
  return local_signed_distance(box, pose.inverse() * p - box.center_offset, q);
}

std::optional<Contact> box_contact(const RoundedBox& box, const Eigen::Isometry3d& pose, const Eigen::Vector3d& p)
{
  const Eigen::Vector3d local = pose.inverse() * p - box.center_offset;
  Eigen::Vector3d q;
  const double signed_dist = local_signed_distance(box, local, q);
  if (signed_dist >= 0.0) {
    return std::nullopt;
  }
  const Eigen::Vector3d outside = q.cwiseMax(0.0);
  const double outside_dist = outside.norm();

  const Eigen::Vector3d sign = local.unaryExpr([](double v) { return v >= 0.0 ? 1.0 : -1.0; });
  Eigen::Vector3d n_local = Eigen::Vector3d::Zero();
  if (outside_dist > 1e-6) {
    n_local = sign.cwiseProduct(outside / outside_dist);  // fuera de la caja interior: hacia el punto más cercano
  } else {
    Eigen::Index axis;
    q.maxCoeff(&axis);  // dentro de la caja interior: por la cara más cercana
    n_local[axis] = sign[axis];
  }
  const double depth = -signed_dist;
  const Eigen::Vector3d normal = pose.linear() * n_local;
  return Contact{depth, normal, p + depth * normal};
}

std::optional<Contact> box_contact_along(const RoundedBox& box, const Eigen::Isometry3d& pose, const Eigen::Vector3d& p,
                                         const Eigen::Vector3d& exit_local)
{
  const Eigen::Vector3d local = pose.inverse() * p - box.center_offset;
  Eigen::Vector3d q;
  if (local_signed_distance(box, local, q) >= 0.0) {
    return std::nullopt;
  }
  // La caja es convexa: desde un punto de dentro, la recta en exit_local cruza la superficie una sola vez.
  // A 2 diagonales ya se está fuera seguro; se busca el cruce por bisección.
  double lo = 0.0, hi = 4.0 * box.half_extents.norm();
  for (int it = 0; it < 24; ++it) {  // 2^-24 de 2 diagonales: < 1 µm
    const double mid = 0.5 * (lo + hi);
    (local_signed_distance(box, local + mid * exit_local, q) < 0.0 ? lo : hi) = mid;
  }
  const Eigen::Vector3d normal = pose.linear() * exit_local;
  return Contact{hi, normal, p + hi * normal};
}

bool point_in_box_bounds(const RoundedBox& box, const Eigen::Isometry3d& pose, const Eigen::Vector3d& p)
{
  const Eigen::Vector3d local = pose.inverse() * p - box.center_offset;
  return (local.cwiseAbs() - box.half_extents).maxCoeff() <= 0.0;
}

bool segment_in_box_bounds(const RoundedBox& box, const Eigen::Isometry3d& pose, const Eigen::Vector3d& p0,
                           const Eigen::Vector3d& p1)
{
  // Prueba de franjas: el tramo de t ∈ [0, 1] que queda entre las dos caras de cada eje, y su intersección
  const Eigen::Isometry3d to_local = pose.inverse();
  const Eigen::Vector3d a = to_local * p0 - box.center_offset;
  const Eigen::Vector3d d = to_local * p1 - box.center_offset - a;
  double lo = 0.0, hi = 1.0;
  for (int axis = 0; axis < 3; ++axis) {
    const double h = box.half_extents[axis];
    if (std::abs(d[axis]) < 1e-12) {
      if (std::abs(a[axis]) > h) {
        return false;
      }
      continue;
    }
    double t0 = (-h - a[axis]) / d[axis], t1 = (h - a[axis]) / d[axis];
    if (t0 > t1) {
      std::swap(t0, t1);
    }
    lo = std::max(lo, t0);
    hi = std::min(hi, t1);
    if (lo > hi) {
      return false;
    }
  }
  return true;
}

double deepest_on_segment(const RoundedBox& box, const Eigen::Isometry3d& pose, const Eigen::Vector3d& p0,
                          const Eigen::Vector3d& p1)
{
  // En el marco de la caja una sola vez: el segmento sigue siendo un segmento
  const Eigen::Isometry3d to_local = pose.inverse();
  const Eigen::Vector3d a = to_local * p0 - box.center_offset;
  const Eigen::Vector3d d = to_local * p1 - box.center_offset - a;
  Eigen::Vector3d q;
  auto f = [&](double t) { return local_signed_distance(box, a + t * d, q); };

  constexpr double kInvPhi = 0.6180339887498949;  // 1/φ
  double lo = 0.0, hi = 1.0;
  double x1 = hi - kInvPhi * (hi - lo), x2 = lo + kInvPhi * (hi - lo);
  double f1 = f(x1), f2 = f(x2);
  for (int it = 0; it < 20; ++it) {  // 0,618^20 ≈ 7e-5 del largo del tramo: unos µm
    if (f1 <= f2) {
      hi = x2;
      x2 = x1;
      f2 = f1;
      x1 = hi - kInvPhi * (hi - lo);
      f1 = f(x1);
    } else {
      lo = x1;
      x1 = x2;
      f1 = f2;
      x2 = lo + kInvPhi * (hi - lo);
      f2 = f(x2);
    }
  }
  return 0.5 * (lo + hi);
}

}  // namespace stonefish_net::core
