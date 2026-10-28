#include "stonefish_net/core/jet.hpp"

#include <algorithm>
#include <cmath>

namespace stonefish_net::core
{

std::optional<Jet> jet_from_thrust(const Eigen::Vector3d& center, const Eigen::Vector3d& thrust_axis, double thrust,
                                   double diameter, double rho)
{
  if (std::abs(thrust) < 1e-6 || diameter <= 0.0 || !std::isfinite(thrust) || !center.allFinite()) {
    return std::nullopt;
  }
  const double area = M_PI * diameter * diameter / 4.0;
  Jet j;
  j.origin = center.cast<float>();
  j.dir = (thrust > 0.0 ? -thrust_axis : thrust_axis).normalized().cast<float>();
  j.speed = static_cast<float>(std::sqrt(2.0 * std::abs(thrust) / (rho * area)));
  j.radius = static_cast<float>(diameter / (2.0 * std::sqrt(2.0)));
  return j;
}

Eigen::Vector3f jet_velocity(const Jet& jet, const Eigen::Vector3f& p, float spread)
{
  const Eigen::Vector3f r = p - jet.origin;
  const float s = r.dot(jet.dir);
  const float d0 = 2.f * jet.radius;
  if (s <= 0.f || s > kJetLength * d0) {
    return Eigen::Vector3f::Zero();
  }
  const float radial = (r - s * jet.dir).norm();
  const float core = kJetCore * d0;
  const float axis = s <= core ? jet.speed : jet.speed * core / s;
  const float b = std::max(jet.radius, spread * s);
  return axis * std::exp(-(radial / b) * (radial / b)) * jet.dir;
}

Eigen::Vector3f jet_velocity(const std::vector<Jet>& jets, const Eigen::Vector3f& p, float spread)
{
  Eigen::Vector3f v = Eigen::Vector3f::Zero();
  for (const Jet& j : jets) v += jet_velocity(j, p, spread);
  return v;
}

}  // namespace stonefish_net::core
