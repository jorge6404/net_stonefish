#include "stonefish_net/NetSleep.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <BulletSoftBody/btSoftBody.h>

#include "convert.h"

namespace stonefish_net
{

NetSleep::NetSleep(const core::NetTopology& topo, const core::NetConfig::Sleep& cfg)
: cfg_(cfg),
  tiles_(cfg.enabled ? core::partition_tiles(topo, cfg.tile_size) : core::NetTiles{}),
  state_(tiles_.tiles.size()),
  frozen_(tiles_.tiles.size()),
  im_(topo.nodes.size(), 0)
{
  if (tiles_.tile_of.empty()) {
    tiles_.tile_of.assign(topo.nodes.size(), -1);
  }
  for (const auto& l : topo.links) links_.emplace_back(l.a, l.b);
  rebuild();
}

void NetSleep::update(btSoftBody& body, double dt, const std::vector<RobotContact::Sphere>& robots)
{
  bool changed = false;
  for (size_t t = 0; t < tiles_.tiles.size(); ++t) {
    const core::Tile& tile = tiles_.tiles[t];
    if (state_[t].awake) {
      Eigen::Vector3d lo = Eigen::Vector3d::Constant(std::numeric_limits<double>::max()), hi = -lo;
      double speed2 = 0.0;
      for (int k : tile.nodes) {
        const Eigen::Vector3d x = eig(body.m_nodes[k].m_x);
        lo = lo.cwiseMin(x);
        hi = hi.cwiseMax(x);
        speed2 = std::max(speed2, static_cast<double>(body.m_nodes[k].m_v.length2()));
      }
      const Eigen::Vector3d center = 0.5 * (lo + hi);
      const double radius = 0.5 * (hi - lo).norm();
      if (core::step_tile(state_[t], cfg_, dt, gap(center, radius, robots), std::sqrt(speed2), 0.0)) {
        frozen_[t].center = center;
        frozen_[t].radius = radius;
        freeze(body, t);
        changed = true;
      }
      continue;
    }
    const Frozen& f = frozen_[t];
    double shift2 = 0.0;
    for (const auto& [k, x0] : f.outside) {
      shift2 = std::max(shift2, (eig(body.m_nodes[k].m_x) - x0).squaredNorm());
    }
    if (core::step_tile(state_[t], cfg_, dt, gap(f.center, f.radius, robots), 0.0, std::sqrt(shift2))) {
      wake(body, t);
      changed = true;
    }
  }
  if (changed) {
    body.updateLinkConstants();  // masa inversa de los extremos de cada tramo (m_c0); no toca su longitud
    rebuild();
  }
}

double NetSleep::gap(const Eigen::Vector3d& center, double radius,
                     const std::vector<RobotContact::Sphere>& robots) const
{
  double g = std::numeric_limits<double>::infinity();
  for (const auto& r : robots) g = std::min(g, (center - r.center).norm() - radius - r.radius);
  return g;
}

void NetSleep::freeze(btSoftBody& body, size_t t)
{
  for (int k : tiles_.tiles[t].nodes) {
    btSoftBody::Node& n = body.m_nodes[k];
    im_[static_cast<size_t>(k)] = n.m_im;
    n.m_im = 0;
    n.m_v = btVector3(0, 0, 0);
    n.m_f = btVector3(0, 0, 0);
  }
  auto& outside = frozen_[t].outside;
  outside.clear();
  for (int j : tiles_.tiles[t].boundary) {
    const auto [a, b] = links_[static_cast<size_t>(j)];
    const int k = tiles_.tile_of[static_cast<size_t>(a)] == static_cast<int>(t) ? b : a;
    outside.emplace_back(k, eig(body.m_nodes[k].m_x));
  }
}

void NetSleep::wake(btSoftBody& body, size_t t)
{
  for (int k : tiles_.tiles[t].nodes) {
    btSoftBody::Node& n = body.m_nodes[k];
    n.m_im = static_cast<btScalar>(im_[static_cast<size_t>(k)]);
    n.m_q = n.m_x;
    n.m_v = btVector3(0, 0, 0);
  }
}

void NetSleep::rebuild()
{
  auto asleep = [&](int k) {
    const int t = tiles_.tile_of[static_cast<size_t>(k)];
    return t >= 0 && !state_[static_cast<size_t>(t)].awake;
  };
  active_nodes_.clear();
  for (int k = 0; k < static_cast<int>(tiles_.tile_of.size()); ++k) {
    if (!asleep(k)) active_nodes_.push_back(k);
  }
  active_links_.clear();
  for (int j = 0; j < static_cast<int>(links_.size()); ++j) {
    if (!asleep(links_[static_cast<size_t>(j)].first) || !asleep(links_[static_cast<size_t>(j)].second)) {
      active_links_.push_back(j);
    }
  }
  awake_ = static_cast<int>(std::count_if(state_.begin(), state_.end(), [](const auto& s) { return s.awake; }));
}

}  // namespace stonefish_net
