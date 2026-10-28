#include "stonefish_net/core/net_topology.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

#include "topology/damage.hpp"
#include "topology/graph.hpp"
#include "topology/lattice.hpp"
#include "topology/rigging.hpp"

namespace stonefish_net::core
{
namespace
{
constexpr double kLeadDensity = 11340.0;

// Masa y volumen de cada nudo con los tramos que quedan (la mitad de cada tramo a cada extremo), la esfera del nudo
// en los nudos de hilo (del mismo material), los plomos y la masa añadida del agua.
void assign_masses(const NetConfig& c, NetTopology& t)
{
  const double knot_volume = M_PI * std::pow(c.net.knot_diameter, 3) / 6.0;
  for (const auto& l : t.links) {
    if (l.kind != LinkKind::Twine) continue;
    for (int n : {l.a, l.b}) {
      Node& node = t.nodes[static_cast<size_t>(n)];
      if (node.kind == NodeKind::Knot && node.knot_diameter == 0.0 && c.net.knot_diameter > 0.0) {
        node.knot_diameter = c.net.knot_diameter;
        node.volume += knot_volume;
        node.mass += knot_volume * c.net.twine_density;
      }
    }
  }
  for (const auto& l : t.links) {
    const double rho = l.kind == LinkKind::Twine ? c.net.twine_density : c.frame_rope.density;
    const double vol = M_PI * l.diameter * l.diameter / 4.0 * l.rest_length;
    for (int n : {l.a, l.b}) {
      t.nodes[n].volume += vol / 2.0;
      t.nodes[n].mass += vol * rho / 2.0;
    }
  }
  for (int n : t.marks.leads) {
    t.nodes[n].mass += c.leads.mass;
    t.nodes[n].volume += c.leads.mass / kLeadDensity;
  }
  for (auto& n : t.nodes) {
    n.inertial_mass = n.mass + c.hydro.added_mass * c.hydro.water_density * n.volume;
  }
}

// Flexibilidad de cada tramo, L / (E · A), con el módulo del hilo o el del cabo (inf → 0, inextensible)
void assign_compliance(const NetConfig& c, NetTopology& t)
{
  for (auto& l : t.links) {
    const double modulus = l.kind == LinkKind::Twine ? c.net.twine_modulus : c.frame_rope.modulus;
    l.compliance = l.rest_length / (modulus * M_PI * l.diameter * l.diameter / 4.0);
  }
}
}  // namespace

void Landmarks::remap(const std::vector<int>& old_to_new)
{
  for (auto& b : buoys) b.node = old_to_new[b.node];
  for (auto& l : leads) l = old_to_new[l];
  for (auto& m : moorings) m.corner = old_to_new[m.corner];
}

NetTopology build_topology(const NetConfig& cfg)
{
  using namespace topology;
  NetTopology t;
  const Lattice lat = build_lattice(cfg, t);
  add_frame(cfg, lat, t);
  t.marks.leads = along_edge(lat, lat.nJ, cfg.leads.spacing);
  if (cfg.buoys.enabled) add_droppers(cfg, lat, t);
  add_moorings(cfg, lat, t);
  add_damage(cfg, lat, t);
  prune_to_component(t, lat.corner(false, false));  // esquina del marco: siempre en la pieza principal
  assign_masses(cfg, t);
  assign_compliance(cfg, t);
  return t;
}

Adjacency node_links(const NetTopology& topo)
{
  Adjacency adj;
  adj.begin.assign(topo.nodes.size() + 1, 0);
  for (const auto& l : topo.links) {
    ++adj.begin[static_cast<size_t>(l.a) + 1];
    ++adj.begin[static_cast<size_t>(l.b) + 1];
  }
  std::partial_sum(adj.begin.begin(), adj.begin.end(), adj.begin.begin());
  adj.links.resize(static_cast<size_t>(adj.begin.back()));
  std::vector<int> next(adj.begin.begin(), adj.begin.end() - 1);
  for (size_t j = 0; j < topo.links.size(); ++j) {
    adj.links[static_cast<size_t>(next[static_cast<size_t>(topo.links[j].a)]++)] = static_cast<int>(j);
    adj.links[static_cast<size_t>(next[static_cast<size_t>(topo.links[j].b)]++)] = static_cast<int>(j);
  }
  return adj;
}

std::vector<int> dropper_nodes(const NetTopology& topo, int mount)
{
  const Adjacency adj = node_links(topo);
  std::vector<int> out{mount}, todo{mount};
  while (!todo.empty()) {
    const auto k = static_cast<size_t>(todo.back());
    todo.pop_back();
    for (int a = adj.begin[k]; a < adj.begin[k + 1]; ++a) {
      const Link& l = topo.links[static_cast<size_t>(adj.links[static_cast<size_t>(a)])];
      const int other = l.a == static_cast<int>(k) ? l.b : l.a;
      if (l.kind == LinkKind::Dropper && topo.nodes[static_cast<size_t>(other)].kind == NodeKind::Dropper &&
          std::find(out.begin(), out.end(), other) == out.end()) {
        out.push_back(other);
        todo.push_back(other);
      }
    }
  }
  return out;
}

}  // namespace stonefish_net::core
