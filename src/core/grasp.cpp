#include "stonefish_net/core/grasp.hpp"

#include <algorithm>
#include <utility>

namespace stonefish_net::core
{
namespace
{
bool finite(const Eigen::Isometry3d& pose) { return pose.matrix().allFinite(); }

// Centro de la caja del dedo en el mundo: el origen de su marco suele estar en la articulación, que no se mueve al
// abrir una pinza de dedos que giran
Eigen::Vector3d center(const Finger& f) { return f.pose * f.box.center_offset; }

// Caja alineada con el mundo que contiene las cajas de los dedos finitos de una pinza, más `margin`
struct Bounds
{
  Eigen::Vector3d lo = Eigen::Vector3d::Constant(1e30), hi = Eigen::Vector3d::Constant(-1e30);
  bool contains(const Eigen::Vector3d& p) const
  {
    return (p.array() >= lo.array()).all() && (p.array() <= hi.array()).all();
  }
};
Bounds bounds(const std::vector<Finger>& fingers, double margin)
{
  Bounds b;
  for (const auto& f : fingers) {
    if (!finite(f.pose)) continue;
    const Eigen::Vector3d c = center(f);
    const Eigen::Vector3d half = f.pose.linear().cwiseAbs() * f.box.half_extents + Eigen::Vector3d::Constant(margin);
    b.lo = b.lo.cwiseMin(c - half);
    b.hi = b.hi.cwiseMax(c + half);
  }
  return b;
}
}  // namespace

Grasp::Grasp(const NetTopology& topo, std::vector<Gripper> grippers, std::vector<int> anchored)
: topo_(topo),
  adj_(node_links(topo)),
  grippers_(std::move(grippers)),
  blocked_(topo.nodes.size(), false),
  owner_(topo.nodes.size(), -1),
  start_(grippers_.size())
{
  for (size_t k = 0; k < topo.nodes.size(); ++k) blocked_[k] = topo.nodes[k].fixed;
  for (int k : anchored) blocked_[static_cast<size_t>(k)] = true;
}

bool Grasp::clear_of(int k, const std::vector<Eigen::Vector3f>& x, const std::vector<Finger>& fingers,
                     double margin) const
{
  const auto node = static_cast<size_t>(k);
  const Eigen::Vector3d p = x[node].cast<double>();
  int pinching = 0;
  for (const auto& f : fingers) {
    if (!finite(f.pose)) return false;
    const double d = box_signed_distance(f.box, f.pose, p);
    if (!(d > 0.0) || (d <= margin && ++pinching == 2)) return false;
    for (int a = adj_.begin[node]; a < adj_.begin[node + 1]; ++a) {
      const Link& l = topo_.links[static_cast<size_t>(adj_.links[static_cast<size_t>(a)])];
      const Eigen::Vector3d q = x[static_cast<size_t>(l.a == k ? l.b : l.a)].cast<double>();
      if (!segment_in_box_bounds(f.box, f.pose, p, q)) continue;
      const double u = deepest_on_segment(f.box, f.pose, p, q);
      if (!(box_signed_distance(f.box, f.pose, p + u * (q - p)) > 0.0)) return false;
    }
  }
  return true;
}

std::vector<GraspEvent> Grasp::update(const std::vector<Eigen::Vector3f>& x, const std::vector<Finger>& bodies)
{
  const size_t count = grippers_.size();
  // Los dedos de cada pinza, en su orden
  std::vector<std::vector<Finger>> fingers(count);
  for (size_t g = 0; g < count; ++g) {
    start_[g].clear();
    for (size_t b : grippers_[g].fingers) {
      fingers[g].push_back(bodies[b]);
      start_[g].push_back(bodies[b].pose);
    }
  }
  std::vector<int> grabbed(count, 0), released(count, 0);

  // Suelta: la pareja que agarró se ha abierto más de release_gap
  holds_.erase(std::remove_if(holds_.begin(), holds_.end(),
                              [&](const Hold& h) {
                                const auto& f = fingers[h.gripper];
                                if (!finite(f[h.finger].pose) || !finite(f[h.partner].pose)) return false;
                                const double gap = (center(f[h.finger]) - center(f[h.partner])).norm();
                                if (!(gap > h.gap + grippers_[h.gripper].release_gap)) return false;
                                owner_[static_cast<size_t>(h.node)] = -1;
                                ++released[h.gripper];
                                releasing_.emplace_back(h.node, h.gripper);
                                return true;
                              }),
               holds_.end());

  // Los soltados vuelven a tocar los dedos de su pinza cuando ya no los solapan
  releasing_.erase(std::remove_if(releasing_.begin(), releasing_.end(),
                                  [&](const std::pair<int, size_t>& r) {
                                    return clear_of(r.first, x, fingers[r.second], grippers_[r.second].grip_margin);
                                  }),
                   releasing_.end());

  // Agarra: lo pellizcado entre dos dedos de la misma pinza
  for (size_t g = 0; g < count; ++g) {
    const auto& f = fingers[g];
    const double margin = grippers_[g].grip_margin;
    const Bounds near = bounds(f, margin);
    // Los dos primeros dedos (en su orden) a menos de margin de p
    const auto pinch = [&](const Eigen::Vector3d& p, size_t& first, size_t& second) {
      int found = 0;
      for (size_t i = 0; i < f.size() && found < 2; ++i) {
        if (finite(f[i].pose) && box_signed_distance(f[i].box, f[i].pose, p) <= margin) {
          (found == 0 ? first : second) = i;
          ++found;
        }
      }
      return found == 2;
    };
    const auto takes = [&](int k) {
      return grabbable(k) && std::find(releasing_.begin(), releasing_.end(), std::pair<int, size_t>(k, g)) == releasing_.end();
    };
    const auto hold = [&](int k, size_t a, size_t b) {
      const Eigen::Vector3d p = x[static_cast<size_t>(k)].cast<double>();
      holds_.push_back({k, g, a, b, f[a].pose.inverse() * p, (center(f[a]) - center(f[b])).norm()});
      owner_[static_cast<size_t>(k)] = static_cast<int>(g);
      ++grabbed[g];
    };
    size_t a = 0, b = 0;
    for (size_t k = 0; k < x.size(); ++k) {
      const Eigen::Vector3d p = x[k].cast<double>();
      if (takes(static_cast<int>(k)) && near.contains(p) && pinch(p, a, b)) hold(static_cast<int>(k), a, b);
    }
    for (const auto& l : topo_.links) {
      if (!takes(l.a) || !takes(l.b)) continue;
      const Eigen::Vector3d mid = 0.5 * (x[static_cast<size_t>(l.a)] + x[static_cast<size_t>(l.b)]).cast<double>();
      if (near.contains(mid) && pinch(mid, a, b)) {
        hold(l.a, a, b);
        hold(l.b, a, b);
      }
    }
  }

  std::vector<GraspEvent> events;
  for (size_t g = 0; g < count; ++g) {
    if (grabbed[g] == 0 && released[g] == 0) continue;
    events.push_back({g, grabbed[g], released[g], static_cast<int>(held(g).size())});
  }
  return events;
}

std::vector<GripAnchor> Grasp::anchors(const std::vector<Eigen::Isometry3d>& end) const
{
  std::vector<GripAnchor> out;
  for (const auto& h : holds_) {
    const size_t body = grippers_[h.gripper].fingers[h.finger];
    const Eigen::Isometry3d& from = start_[h.gripper][h.finger];
    const Eigen::Isometry3d& to = end[body];
    if (!finite(from) || !finite(to)) continue;
    Anchor a;
    a.node = h.node;
    a.from = (from * h.local).cast<float>();
    a.to = (to * h.local).cast<float>();
    out.push_back({a, body});
  }
  return out;
}

void Grasp::exclude_held(std::vector<ContactBody>& bodies) const
{
  for (size_t g = 0; g < grippers_.size(); ++g) {
    const std::vector<int> nodes = excluded(g);
    for (size_t b : grippers_[g].fingers) {
      bodies[b].excluded_nodes.insert(bodies[b].excluded_nodes.end(), nodes.begin(), nodes.end());
    }
  }
}

std::vector<int> Grasp::held(size_t gripper) const
{
  std::vector<int> out;
  for (const auto& h : holds_) {
    if (h.gripper == gripper) out.push_back(h.node);
  }
  return out;
}

std::vector<int> Grasp::excluded(size_t gripper) const
{
  std::vector<int> out = held(gripper);
  for (const auto& [node, g] : releasing_) {
    if (g == gripper && std::find(out.begin(), out.end(), node) == out.end()) out.push_back(node);
  }
  return out;
}

}  // namespace stonefish_net::core
