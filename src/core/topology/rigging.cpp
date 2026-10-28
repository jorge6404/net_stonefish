#include "rigging.hpp"

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "graph.hpp"

namespace stonefish_net::core::topology
{
namespace
{
constexpr double kAnchorClearance = 0.01;  // m: el nudo fijo va sobre el bloque, no dentro (la colisión lo expulsaba)

// segs + 1 puntos equiespaciados en t ∈ [0, 1] de la curva p(t) = p0 + t (p1 − p0) + a sin(π t) n.
std::vector<Eigen::Vector3d> sine_curve(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const Eigen::Vector3d& n,
                                        double a, int segs)
{
  std::vector<Eigen::Vector3d> p(segs + 1);
  for (int k = 0; k <= segs; ++k) {
    const double t = static_cast<double>(k) / segs;
    p[k] = p0 + t * (p1 - p0) + a * std::sin(M_PI * t) * n;
  }
  return p;
}

double polyline_length(const std::vector<Eigen::Vector3d>& p)
{
  double l = 0;
  for (size_t k = 1; k < p.size(); ++k) l += (p[k] - p[k - 1]).norm();
  return l;
}

// segs + 1 puntos de p0 a p1 sobre una media onda de longitud exacta `length`, abombada hacia arriba
// (−z, lejos del suelo) en el plano vertical. Bullet toma la longitud natural de cada tramo de su
// distancia inicial: una cadena recta no tendría holgura, así que la holgura va en la forma.
std::vector<Eigen::Vector3d> slack_curve(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, double length, int segs)
{
  const Eigen::Vector3d u = (p1 - p0).normalized();
  const Eigen::Vector3d up_dir(0, 0, -1);
  Eigen::Vector3d n = up_dir - up_dir.dot(u) * u;
  if (n.norm() < 1e-9) {
    n = Eigen::Vector3d(0, 1, 0) - Eigen::Vector3d(0, 1, 0).dot(u) * u;
  }
  n.normalize();
  double lo = 0.0, hi = length;  // con amplitud = length la curva ya es más larga que length
  for (int it = 0; it < 200; ++it) {
    const double mid = 0.5 * (lo + hi);
    (polyline_length(sine_curve(p0, p1, n, mid, segs)) < length ? lo : hi) = mid;
  }
  return sine_curve(p0, p1, n, 0.5 * (lo + hi), segs);
}

// Cadena de cabo que sale de `from` (= pts[0]) por pts[1..]; devuelve su último nudo.
int add_chain(const NetConfig& c, NetTopology& t, int from, const std::vector<Eigen::Vector3d>& pts, NodeKind node_kind,
              LinkKind link_kind)
{
  Strand strand{{from}, link_kind};
  for (size_t k = 1; k < pts.size(); ++k) {
    const int n = add_node(t, pts[k], node_kind);
    add_link(t, strand.nodes.back(), n, c.frame_rope.diameter, link_kind);
    strand.nodes.push_back(n);
  }
  t.strands.push_back(std::move(strand));
  return t.strands.back().nodes.back();
}

int segments(double length, double bar) { return static_cast<int>(std::ceil(length / bar - 1e-9)); }
}  // namespace

void add_droppers(const NetConfig& c, const Lattice& lat, NetTopology& t)
{
  const int segs = segments(c.buoys.dropper, c.net.bar);
  const double step = c.buoys.dropper / segs;
  for (int base : along_edge(lat, 0, c.buoys.spacing)) {
    std::vector<Eigen::Vector3d> pts(segs + 1);
    for (int k = 0; k <= segs; ++k) {
      pts[k] = t.nodes[base].position - Eigen::Vector3d(0, 0, k * step);
    }
    const int top = add_chain(c, t, base, pts, NodeKind::Dropper, LinkKind::Dropper);
    t.marks.buoys.push_back(BuoyMount{top, t.nodes[top].position});
  }
}

void add_moorings(const NetConfig& c, const Lattice& lat, NetTopology& t)
{
  for (bool top : {false, true}) {  // orden de Landmarks::moorings: inferiores y luego superiores, izq. y der.
    const NetConfig::Line& line = top ? c.mooring.top : c.mooring.bottom;
    if (line.to == NetConfig::LineTo::None) continue;
    const bool on_floor = line.to == NetConfig::LineTo::Floor;
    for (bool right : {false, true}) {
      const int corner = lat.corner(right, !top);
      const Eigen::Vector3d p0 = t.nodes[corner].position;
      const double side = right ? 1.0 : -1.0;
      Eigen::Vector3d base = Eigen::Vector3d::Zero(), anchor;
      if (on_floor) {
        base = Eigen::Vector3d(p0.x() - c.mooring.upstream, p0.y() + side * line.outward, c.mooring.floor_z);
        anchor = base - Eigen::Vector3d(0, 0, c.mooring.block_height + kAnchorClearance);
        // la línea tiene que bajar hasta el ancla: si no, sus últimos nudos arrancan dentro del bloque
        if (!(anchor.z() > p0.z())) {
          throw std::invalid_argument("mooring.floor_z: el ancla (z=" + std::to_string(anchor.z()) +
                                      ") queda por encima de la esquina de la red (z=" + std::to_string(p0.z()) +
                                      "); baja floor_z o block_height");
        }
      } else {
        // a kAnchorClearance de la pared, por dentro del tanque
        anchor = Eigen::Vector3d(p0.x(), side * (c.mooring.wall_y - kAnchorClearance), c.mooring.wall_z);
        if (!(side * (anchor.y() - p0.y()) > 0)) {
          throw std::invalid_argument("mooring.wall_y: la pared (y=±" + std::to_string(c.mooring.wall_y) +
                                      ") no queda por fuera de la esquina de la red (y=" + std::to_string(p0.y()) +
                                      "); sube wall_y o estrecha la red");
        }
      }
      const double length = (anchor - p0).norm() * line.slack;
      const int end = add_chain(c, t, corner, slack_curve(p0, anchor, length, segments(length, c.net.bar)),
                                NodeKind::Mooring, LinkKind::Mooring);
      t.nodes[end].fixed = true;
      t.marks.moorings.push_back(MooringLine{corner, on_floor, base});
    }
  }
}

}  // namespace stonefish_net::core::topology
