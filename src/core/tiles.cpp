#include "stonefish_net/core/tiles.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace stonefish_net::core
{
namespace
{
bool sleeps(const Node& n) { return n.kind == NodeKind::Knot && !n.fixed; }
}  // namespace

NetTiles partition_tiles(const NetTopology& topo, double tile_size)
{
  NetTiles out;
  out.tile_of.assign(topo.nodes.size(), -1);
  double y0 = std::numeric_limits<double>::max(), z0 = y0;
  for (const auto& n : topo.nodes) {
    if (sleeps(n)) {
      y0 = std::min(y0, n.position.y());
      z0 = std::min(z0, n.position.z());
    }
  }

  // Celda (iy, iz) → baldosa, en orden de aparición
  std::vector<std::pair<long, long>> cells;
  for (size_t k = 0; k < topo.nodes.size(); ++k) {
    const auto& n = topo.nodes[k];
    if (!sleeps(n)) {
      continue;
    }
    const std::pair<long, long> cell{std::lround(std::floor((n.position.y() - y0) / tile_size)),
                                     std::lround(std::floor((n.position.z() - z0) / tile_size))};
    auto it = std::find(cells.begin(), cells.end(), cell);
    if (it == cells.end()) {
      cells.push_back(cell);
      out.tiles.emplace_back();
      it = cells.end() - 1;
    }
    const int t = static_cast<int>(it - cells.begin());
    out.tile_of[k] = t;
    out.tiles[static_cast<size_t>(t)].nodes.push_back(static_cast<int>(k));
  }

  for (size_t j = 0; j < topo.links.size(); ++j) {
    const int ta = out.tile_of[static_cast<size_t>(topo.links[j].a)];
    const int tb = out.tile_of[static_cast<size_t>(topo.links[j].b)];
    if (ta == tb) {
      continue;
    }
    for (int t : {ta, tb}) {
      if (t >= 0) {
        out.tiles[static_cast<size_t>(t)].boundary.push_back(static_cast<int>(j));
      }
    }
  }
  return out;
}

bool step_tile(TileState& s, const NetConfig::Sleep& cfg, double dt, double gap, double max_speed, double max_shift)
{
  if (s.awake) {
    s.calm = gap > cfg.sleep_radius && max_speed < cfg.sleep_speed ? s.calm + dt : 0.0;
    if (s.calm < cfg.settle_time) {
      return false;
    }
    s.awake = false;
    s.calm = 0.0;
    return true;
  }
  if (gap < cfg.wake_radius || max_shift > cfg.wake_shift) {
    s.awake = true;
    return true;
  }
  return false;
}

}  // namespace stonefish_net::core
