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
                                             static_cast<float>(n.inertial_mass),
                                             static_cast<float>(n.knot_diameter)});
  }
  const size_t links = topo.links.size();
  std::vector<int> new_index(links);
  for (size_t i = 0; i < coloring.order.size(); ++i) {
    const auto j = static_cast<size_t>(coloring.order[i]);
    const bool shear = j >= links;
    const auto& l = shear ? topo.shear[j - links] : topo.links[j];
    if (!shear) new_index[j] = static_cast<int>(i);
    L.links.insert(L.links.end(), {l.a, l.b});
    L.link_props.insert(L.link_props.end(), {static_cast<float>(l.rest_length), static_cast<float>(l.diameter),
                                             static_cast<float>(l.compliance), shear ? 1.f : 0.f});
  }
  const Adjacency adj = node_links(topo);
  L.adj_begin = adj.begin;
  L.adj_links.reserve(adj.links.size());
  for (int j : adj.links) L.adj_links.push_back(new_index[static_cast<size_t>(j)]);
  L.color_begin = coloring.color_begin;
  L.contact_colors = coloring.contact_colors;
  return L;
}

}  // namespace stonefish_net::core
