#include "stonefish_net/core/net_topology.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <set>
#include <utility>

#include "topology/damage.hpp"
#include "topology/graph.hpp"
#include "topology/irregular.hpp"
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

// Rigidez del ángulo del nudo: las dos diagonales de cada rombo de hilo (ciclo nudo–hilo–nudo–hilo de 4 nudos). En
// un rombo de lado b y semiángulo θ, d_y = 2b sin θ y d_z = 2b cos θ; un cambio Δφ de su ángulo las cambia en
// b cos θ · Δφ y b sin θ · Δφ. Con rigidez 8k / d_otra² en cada una, el rombo guarda ½ (4k) Δφ²: la de sus cuatro
// esquinas, cada una con rigidez k. Reposo: el ángulo de la red al construirla (opening_deg), cambiado por
// irregular.angle_deg, con el lado del rombo de la red construida.
void add_shear(const NetConfig& c, NetTopology& t)
{
  if (c.net.knot_stiffness <= 0.0) return;
  const topology::RandomField field = topology::angle_field(c);
  std::vector<std::vector<int>> twine(t.nodes.size());
  for (const auto& l : t.links) {
    if (l.kind != LinkKind::Twine) continue;
    twine[static_cast<size_t>(l.a)].push_back(l.b);
    twine[static_cast<size_t>(l.b)].push_back(l.a);
  }
  const auto linked = [&](int a, int b) {
    const auto& n = twine[static_cast<size_t>(a)];
    return std::find(n.begin(), n.end(), b) != n.end();
  };
  const auto at = [&](int k) -> const Eigen::Vector3d& { return t.nodes[static_cast<size_t>(k)].position; };
  std::set<std::pair<int, int>> done;
  for (int a = 0; a < static_cast<int>(t.nodes.size()); ++a) {
    const auto& n = twine[static_cast<size_t>(a)];
    for (size_t i = 0; i < n.size(); ++i) {
      for (size_t j = i + 1; j < n.size(); ++j) {
        const int b = n[i], c2 = n[j];
        for (int d : twine[static_cast<size_t>(b)]) {
          if (d == a || !linked(c2, d) || !done.insert(std::minmax(a, d)).second) continue;
          double ad = (at(a) - at(d)).norm(), bc = (at(b) - at(c2)).norm();
          if (ad < 1e-9 || bc < 1e-9) continue;
          // Ángulo en las esquinas de la diagonal más vertical, y de ahí las dos diagonales de reposo
          const Eigen::Vector3d dad = (at(d) - at(a)).cwiseAbs(), mid = 0.5 * (at(a) + at(d));
          const bool ad_vertical = dad.z() > dad.y();
          const double dv = ad_vertical ? ad : bc, dh = ad_vertical ? bc : ad;
          const double side = 0.5 * std::hypot(dv, dh);
          const double angle = topology::rest_angle(c, field, 2.0 * std::atan2(dh, dv), mid.y(), mid.z());
          const double rv = 2.0 * side * std::cos(angle / 2.0), rh = 2.0 * side * std::sin(angle / 2.0);
          ad = ad_vertical ? rv : rh;
          bc = ad_vertical ? rh : rv;
          t.shear.push_back(Link{a, d, ad, 0.0, LinkKind::Shear, bc * bc / (8.0 * c.net.knot_stiffness)});
          if (done.insert(std::minmax(b, c2)).second) {
            t.shear.push_back(Link{b, c2, bc, 0.0, LinkKind::Shear, ad * ad / (8.0 * c.net.knot_stiffness)});
          }
        }
      }
    }
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
  vary_twine_length(cfg, t);
  assign_masses(cfg, t);
  assign_compliance(cfg, t);
  add_shear(cfg, t);
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
