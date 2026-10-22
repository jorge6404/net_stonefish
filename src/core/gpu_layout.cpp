#include "stonefish_net/core/gpu_layout.hpp"

namespace stonefish_net::core
{

GpuLayout make_gpu_layout(const NetTopology& topo, const LinkColoring& coloring)
{
  GpuLayout L;
  for (const auto& n : topo.nodes) {
    const float w = n.fixed ? 0.f : static_cast<float>(1.0 / n.inertial_mass);
    L.x0.insert(L.x0.end(), {static_cast<float>(n.position.x()), static_cast<float>(n.position.y()),
                             static_cast<float>(n.position.z()), w});
    L.node_props.insert(L.node_props.end(), {static_cast<float>(n.mass), static_cast<float>(n.volume),
                                             static_cast<float>(n.inertial_mass), 0.f});
  }
  std::vector<int> new_index(topo.links.size());
  for (size_t i = 0; i < coloring.order.size(); ++i) {
    const int j = coloring.order[i];
    const auto& l = topo.links[static_cast<size_t>(j)];
    new_index[static_cast<size_t>(j)] = static_cast<int>(i);
    L.links.insert(L.links.end(), {l.a, l.b});
    L.link_props.insert(L.link_props.end(), {static_cast<float>(l.rest_length), static_cast<float>(l.diameter), 0.f, 0.f});
  }
  const Adjacency adj = node_links(topo);
  L.adj_begin = adj.begin;
  L.adj_links.reserve(adj.links.size());
  for (int j : adj.links) L.adj_links.push_back(new_index[static_cast<size_t>(j)]);
  L.color_begin = coloring.color_begin;
  return L;
}

}  // namespace stonefish_net::core
