#include "lattice.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "graph.hpp"

namespace stonefish_net::core::topology
{

int Lattice::at(int i, int j) const
{
  if (i < 0 || i > nI || j < 0 || j > nJ) {
    return -1;
  }
  return grid[static_cast<size_t>(j * (nI + 1) + i)];
}

Lattice build_lattice(const NetConfig& c, NetTopology& t)
{
  Lattice lat;
  const double half = c.net.opening_deg * M_PI / 360.0;
  lat.hy = c.net.bar * std::sin(half);
  lat.hz = c.net.bar * std::cos(half);
  lat.nI = std::max(2, 2 * static_cast<int>(std::lround(c.net.width / (2.0 * lat.hy))));
  lat.nJ = std::max(2, 2 * static_cast<int>(std::lround(c.net.height / (2.0 * lat.hz))));
  lat.grid.assign(static_cast<size_t>((lat.nI + 1) * (lat.nJ + 1)), -1);
  const double y0 = c.pose.y_center - lat.nI * lat.hy / 2.0;
  for (int j = 0; j <= lat.nJ; ++j) {
    for (int i = 0; i <= lat.nI; ++i) {
      if ((i + j) % 2 == 0) {
        lat.grid[static_cast<size_t>(j * (lat.nI + 1) + i)] =
          add_node(t, {c.pose.x, y0 + i * lat.hy, c.pose.z_top + j * lat.hz}, NodeKind::Knot);
      }
    }
  }
  for (int j = 0; j <= lat.nJ; ++j) {
    for (int i = 0; i < lat.nI; ++i) {
      const int a = lat.at(i, j);
      if (a < 0) {
        continue;
      }
      for (int dj : {-1, 1}) {
        const int b = lat.at(i + 1, j + dj);
        if (b >= 0) {
          add_link(t, a, b, c.net.twine_diameter, LinkKind::Twine);
        }
      }
    }
  }
  // Hilos continuos: diagonales (+1, +1) y (+1, −1), cada una desde su nudo inicial
  for (int dj : {1, -1}) {
    for (int j = 0; j <= lat.nJ; ++j) {
      for (int i = 0; i <= lat.nI; ++i) {
        if (lat.at(i, j) < 0 || lat.at(i - 1, j - dj) >= 0) {
          continue;
        }
        Strand s{{}, LinkKind::Twine};
        for (int ci = i, cj = j; lat.at(ci, cj) >= 0; ++ci, cj += dj) {
          s.nodes.push_back(lat.at(ci, cj));
        }
        if (s.nodes.size() > 1) {
          t.strands.push_back(std::move(s));
        }
      }
    }
  }
  return lat;
}

void add_frame(const NetConfig& c, const Lattice& lat, NetTopology& t)
{
  std::vector<int> loop;  // borde en sentido horario empezando arriba a la izquierda
  for (int i = 0; i <= lat.nI; i += 2) loop.push_back(lat.at(i, 0));
  for (int j = 2; j <= lat.nJ; j += 2) loop.push_back(lat.at(lat.nI, j));
  for (int i = lat.nI - 2; i >= 0; i -= 2) loop.push_back(lat.at(i, lat.nJ));
  for (int j = lat.nJ - 2; j > 0; j -= 2) loop.push_back(lat.at(0, j));
  Strand s{{}, LinkKind::Frame};
  for (size_t k = 0; k < loop.size(); ++k) {
    const int a = loop[k], b = loop[(k + 1) % loop.size()];
    add_link(t, a, b, c.frame_rope.diameter, LinkKind::Frame);
    s.nodes.push_back(a);
  }
  s.nodes.push_back(loop.front());
  t.strands.push_back(std::move(s));
}

std::vector<int> along_edge(const Lattice& lat, int j, double spacing)
{
  std::vector<int> out;
  const double width = lat.nI * lat.hy;
  // tolerancia de un paso de retícula: la red real es algo más estrecha que la nominal
  for (int k = 0; k * spacing <= width + lat.hy + 1e-9; ++k) {
    int i = static_cast<int>(std::lround(k * spacing / lat.hy));
    i = std::clamp(i - (i % 2), 0, lat.nI);
    if (i + 2 <= lat.nI && std::abs((i + 2) * lat.hy - k * spacing) < std::abs(i * lat.hy - k * spacing)) {
      i += 2;
    }
    const int n = lat.at(i, j);
    if (out.empty() || out.back() != n) {
      out.push_back(n);
    }
  }
  return out;
}

}  // namespace stonefish_net::core::topology
