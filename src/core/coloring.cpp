#include "stonefish_net/core/coloring.hpp"

#include <algorithm>
#include <cstdint>
#include <numeric>
#include <stdexcept>

namespace stonefish_net::core
{

LinkColoring color_links(const NetTopology& topo)
{
  const size_t links = topo.links.size();
  std::vector<int> degree(topo.nodes.size(), 0);
  for (const auto& l : topo.links) {
    ++degree[static_cast<size_t>(l.a)];
    ++degree[static_cast<size_t>(l.b)];
  }
  std::vector<int> by_degree(links);
  std::iota(by_degree.begin(), by_degree.end(), 0);
  auto weight = [&](int j) {
    const auto& l = topo.links[static_cast<size_t>(j)];
    return degree[static_cast<size_t>(l.a)] + degree[static_cast<size_t>(l.b)];
  };
  std::stable_sort(by_degree.begin(), by_degree.end(), [&](int i, int j) { return weight(i) > weight(j); });

  std::vector<uint64_t> used(topo.nodes.size(), 0);  // colores ya tomados en cada nudo
  std::vector<int> color(links, 0);
  int colors = 0;
  for (int j : by_degree) {
    const auto& l = topo.links[static_cast<size_t>(j)];
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

  LinkColoring out;
  out.color_begin.assign(static_cast<size_t>(colors) + 1, 0);
  for (int c : color) ++out.color_begin[static_cast<size_t>(c) + 1];
  std::partial_sum(out.color_begin.begin(), out.color_begin.end(), out.color_begin.begin());
  out.order.resize(links);
  std::vector<int> next(out.color_begin.begin(), out.color_begin.end() - 1);
  for (size_t j = 0; j < links; ++j) {
    out.order[static_cast<size_t>(next[static_cast<size_t>(color[j])]++)] = static_cast<int>(j);
  }
  return out;
}

}  // namespace stonefish_net::core
