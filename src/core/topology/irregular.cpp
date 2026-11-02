#include "irregular.hpp"

#include <algorithm>
#include <cmath>
#include <random>

namespace stonefish_net::core::topology
{

RandomField::RandomField(double scale, unsigned seed)
{
  std::mt19937 rng(seed);
  std::normal_distribution<double> wave(0.0, 1.0 / scale);
  std::uniform_real_distribution<double> phase(0.0, 2.0 * M_PI);
  for (int i = 0; i < kModes; ++i) {
    k_(i, 0) = wave(rng);
    k_(i, 1) = wave(rng);
    phase_(i) = phase(rng);
  }
}

double RandomField::operator()(double y, double z) const
{
  return std::sqrt(2.0 / kModes) * ((k_.col(0) * y + k_.col(1) * z + phase_).array().cos()).sum();
}

void vary_twine_length(const NetConfig& c, NetTopology& t)
{
  if (c.irregular.length <= 0.0) return;
  const RandomField field(c.irregular.scale, static_cast<unsigned>(c.irregular.seed));
  for (auto& l : t.links) {
    if (l.kind != LinkKind::Twine) continue;
    const Eigen::Vector3d mid = 0.5 * (t.nodes[static_cast<size_t>(l.a)].position + t.nodes[static_cast<size_t>(l.b)].position);
    l.rest_length *= std::max(0.5, 1.0 + c.irregular.length * field(mid.y(), mid.z()));
  }
}

RandomField angle_field(const NetConfig& c)
{
  return RandomField(c.irregular.scale, static_cast<unsigned>(c.irregular.seed) + 1u);
}

double rest_angle(const NetConfig& c, const RandomField& field, double angle_rad, double y, double z)
{
  if (c.irregular.angle_deg <= 0.0) return angle_rad;
  const double a = angle_rad + c.irregular.angle_deg * M_PI / 180.0 * field(y, z);
  return std::clamp(a, 10.0 * M_PI / 180.0, 170.0 * M_PI / 180.0);
}

}  // namespace stonefish_net::core::topology
