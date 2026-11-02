#include "stonefish_net/core/coloring.hpp"

#include <algorithm>
#include <cstdint>
#include <numeric>
#include <stdexcept>
#include <tuple>

namespace stonefish_net::core
{
namespace
{
// Los tramos de nudos con más tramos primero
std::vector<int> by_degree(const std::vector<Link>& list, size_t nodes)
{
  std::vector<int> degree(nodes, 0);
  for (const auto& l : list) {
    ++degree[static_cast<size_t>(l.a)];
    ++degree[static_cast<size_t>(l.b)];
  }
  std::vector<int> order(list.size());
  std::iota(order.begin(), order.end(), 0);
  auto weight = [&](int j) {
    const auto& l = list[static_cast<size_t>(j)];
    return degree[static_cast<size_t>(l.a)] + degree[static_cast<size_t>(l.b)];
  };
  std::stable_sort(order.begin(), order.end(), [&](int i, int j) { return weight(i) > weight(j); });
  return order;
}

// Las diagonales de los rombos forman cadenas rectas, horizontales (y) y verticales (z), en la red sin deformar:
// primero las horizontales fila a fila y luego las verticales columna a columna, cada cadena de un extremo al otro.
// Así el voraz alterna dos colores en cada familia y salen 4 (por grado, 6).
std::vector<int> along_chains(const std::vector<Link>& list, const std::vector<Node>& nodes)
{
  struct Key { bool vertical; double across, along; };
  std::vector<Key> key;
  for (const auto& l : list) {
    const Eigen::Vector3d a = nodes[static_cast<size_t>(l.a)].position, b = nodes[static_cast<size_t>(l.b)].position;
    const Eigen::Vector3d mid = 0.5 * (a + b), d = (b - a).cwiseAbs();
    const bool vertical = d.z() > d.y();
    key.push_back({vertical, vertical ? mid.y() : mid.z(), vertical ? mid.z() : mid.y()});
  }
  std::vector<int> order(list.size());
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(order.begin(), order.end(), [&](int i, int j) {
    const Key &p = key[static_cast<size_t>(i)], &q = key[static_cast<size_t>(j)];
    return std::tie(p.vertical, p.across, p.along) < std::tie(q.vertical, q.across, q.along);
  });
  return order;
}

// Voraz sobre una lista de tramos en ese orden: cada uno toma el primer color libre en sus dos nudos. Color de cada
// uno (desde 0) y cuántos colores salen
int color_list(const std::vector<Link>& list, const std::vector<int>& order, size_t nodes, std::vector<int>& color)
{
  std::vector<uint64_t> used(nodes, 0);  // colores ya tomados en cada nudo
  color.assign(list.size(), 0);
  int colors = 0;
  for (int j : order) {
    const auto& l = list[static_cast<size_t>(j)];
    const uint64_t taken = used[static_cast<size_t>(l.a)] | used[static_cast<size_t>(l.b)];
    if (taken == ~uint64_t{0}) {
      throw std::runtime_error("la red necesita más de 64 colores de tramos");
    }
    int c = 0;
    while (taken & (uint64_t{1} << c)) ++c;
    color[static_cast<size_t>(j)] = c;
    used[static_cast<size_t>(l.a)] |= uint64_t{1} << c;
    used[static_cast<size_t>(l.b)] |= uint64_t{1} << c;
    colors = std::max(colors, c + 1);
  }
  return colors;
}
}  // namespace

LinkColoring color_links(const NetTopology& topo)
{
  std::vector<int> color, shear_color;
  const size_t nodes = topo.nodes.size();
  const int link_colors = color_list(topo.links, by_degree(topo.links, nodes), nodes, color);
  const int shear_colors = color_list(topo.shear, along_chains(topo.shear, topo.nodes), nodes, shear_color);
  for (int c : shear_color) color.push_back(link_colors + c);  // detrás de los de los tramos

  LinkColoring out;
  out.contact_colors = link_colors;
  out.color_begin.assign(static_cast<size_t>(link_colors + shear_colors) + 1, 0);
  for (int c : color) ++out.color_begin[static_cast<size_t>(c) + 1];
  std::partial_sum(out.color_begin.begin(), out.color_begin.end(), out.color_begin.begin());
  out.order.resize(color.size());
  std::vector<int> next(out.color_begin.begin(), out.color_begin.end() - 1);
  for (size_t j = 0; j < color.size(); ++j) {
    out.order[static_cast<size_t>(next[static_cast<size_t>(color[j])]++)] = static_cast<int>(j);
  }
  return out;
}

}  // namespace stonefish_net::core
