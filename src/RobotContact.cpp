#include "stonefish_net/RobotContact.h"

#include <algorithm>
#include <utility>

#include <BulletSoftBody/btSoftBody.h>
#include <Stonefish/entities/SolidEntity.h>

#include "convert.h"
#include "pose_now.h"

namespace stonefish_net
{

RobotContact::RobotContact(SolidContacts& solids, const core::NetTopology& topo)
: solids_(solids),
  starting_(topo.nodes.size()),
  excluded_(core::exclusion_masks(solids.bodies(), topo.nodes.size())),
  node_count_(static_cast<int>(topo.nodes.size())),
  frames_(solids.bodies().size()),
  exits_(solids.bodies().size())
{
  for (size_t j = 0; j < topo.links.size(); ++j) {
    const auto& l = topo.links[j];
    segments_.emplace_back(l.a, l.b);
    starting_[static_cast<size_t>(l.a)].push_back(static_cast<int>(j));
    max_segment_ = std::max(max_segment_, l.rest_length);
  }
  // Un grupo por robot; cada sólido suelto, el suyo
  const auto& bodies = solids_.bodies();
  auto group_key = [](const SolidContacts::Body& t) -> const void* {
    return t.robot ? static_cast<const void*>(t.robot) : t.solid;
  };
  for (size_t ti = 0; ti < bodies.size(); ++ti) {
    auto same = std::find_if(groups_.begin(), groups_.end(),
                             [&](const auto& g) { return group_key(bodies[g.front()]) == group_key(bodies[ti]); });
    if (same == groups_.end()) {
      groups_.push_back({ti});
    } else {
      same->push_back(ti);
    }
  }
}

RobotContact::Sphere RobotContact::reach(const SolidContacts::Body& t, const Eigen::Isometry3d& pose) const
{
  // Lo que puede tocar la esfera que contiene la caja durante un subpaso. Un tramo que la toca tiene sus extremos
  // a menos de radio + su longitud del centro (10 % por si se estira, 5 cm por lo que se muevan los nudos en las
  // iteraciones).
  return {pose * t.box.center_offset, t.box.half_extents.norm() + 1.1 * max_segment_ + 0.05};
}

RobotContact::Sphere RobotContact::enclosing(const std::vector<Sphere>& spheres)
{
  Eigen::Vector3d c = Eigen::Vector3d::Zero();
  for (const Sphere& s : spheres) c += s.center;
  c /= static_cast<double>(spheres.size());
  double r = 0.0;
  for (const Sphere& s : spheres) r = std::max(r, (s.center - c).norm() + s.radius);
  return {c, r};
}

std::vector<RobotContact::Sphere> RobotContact::robot_spheres() const
{
  std::vector<Sphere> out, links;
  for (const auto& group : groups_) {
    links.clear();
    for (size_t ti : group) links.push_back(reach(solids_.bodies()[ti], solids_.pose_now(ti)));
    out.push_back(enclosing(links));
  }
  return out;
}

void RobotContact::begin_step(const btSoftBody& body, double dt, const std::vector<int>& nodes)
{
  dt_ = dt;
  for (size_t ti = 0; ti < frames_.size(); ++ti) {
    const SolidContacts::Body& t = solids_.bodies()[ti];
    Frame& f = frames_[ti];
    f.pose = solids_.pose_now(ti);
    f.cg = eig(cg_now(t.solid).getOrigin());
    f.v_cg = eig(t.solid->getLinearVelocity());
    f.w = eig(t.solid->getAngularVelocity());
    f.momentum.setZero();
    f.angular.setZero();
    f.touching.clear();
    f.nodes.clear();
    f.segments.clear();
    const Sphere sphere = reach(t, f.pose);
    f.center = sphere.center;
    f.reach = sphere.radius;
  }
  // Descarte grueso por robot: una sola pasada por la red con la esfera que envuelve las de todos sus enlaces;
  // solo los nudos de dentro se comparan con cada enlace, y los tramos salen de ellos sin recorrer la red.
  for (const auto& group : groups_) {
    group_spheres_.clear();
    for (size_t ti : group) group_spheres_.push_back({frames_[ti].center, frames_[ti].reach});
    const auto [c, r] = enclosing(group_spheres_);
    group_nodes_.clear();
    for (int k : nodes) {
      if ((eig(body.m_nodes[k].m_x) - c).squaredNorm() <= r * r) {
        group_nodes_.push_back(k);
      }
    }
    for (size_t ti : group) {
      Frame& f = frames_[ti];
      for (int k : group_nodes_) {
        if ((eig(body.m_nodes[k].m_x) - f.center).squaredNorm() > f.reach * f.reach) {
          continue;
        }
        if (body.m_nodes[k].m_im > 0) {
          f.nodes.push_back(k);
        }
        const auto& from = starting_[static_cast<size_t>(k)];
        f.segments.insert(f.segments.end(), from.begin(), from.end());
      }
    }
  }
}

void RobotContact::project(btSoftBody& body)
{
  constexpr double kEnd = 1e-3;  // punto más hundido en un extremo del tramo: ya lo trata el nudo
  for (size_t ti = 0; ti < frames_.size(); ++ti) {
    const SolidContacts::Body& t = solids_.bodies()[ti];
    const Frame& f = frames_[ti];
    // Descartes de menos a más caros: esfera que contiene la caja, la caja sin redondear y, solo si pasa los dos,
    // el punto más hundido del tramo
    const double radius2 = t.box.half_extents.squaredNorm();
    for (int k : f.nodes) {
      if (core::excludes(excluded_[static_cast<size_t>(k)], ti)) continue;
      const Eigen::Vector3d x = eig(body.m_nodes[k].m_x);
      if ((x - f.center).squaredNorm() <= radius2 && core::point_in_box_bounds(t.box, f.pose, x)) {
        contact(ti, k, k, k, 0.0, body);
      }
    }
    for (int j : f.segments) {
      const auto [a, b] = segments_[static_cast<size_t>(j)];
      if (core::excludes(excluded_[static_cast<size_t>(a)], ti) || core::excludes(excluded_[static_cast<size_t>(b)], ti)) {
        continue;
      }
      const Eigen::Vector3d xa = eig(body.m_nodes[a].m_x), xb = eig(body.m_nodes[b].m_x);
      const Eigen::Vector3d d = xb - xa;
      const double len2 = d.squaredNorm();
      const double closest = len2 > 0.0 ? std::clamp((f.center - xa).dot(d) / len2, 0.0, 1.0) : 0.0;
      if ((xa + closest * d - f.center).squaredNorm() > radius2 || !core::segment_in_box_bounds(t.box, f.pose, xa, xb)) {
        continue;
      }
      const double s = core::deepest_on_segment(t.box, f.pose, xa, xb);
      if (s > kEnd && s < 1.0 - kEnd) {
        contact(ti, node_count_ + j, a, b, s, body);
      }
    }
  }
}

// Saca de la caja el punto (1 − s)·x_a + s·x_b del tramo a-b (un nudo: a == b). Sale por la cara recordada si
// el punto ya estaba en contacto (en este subpaso o en el anterior); si acaba de entrar, por la superficie más
// cercana, que al entrar es la de entrada. Rozamiento de Coulomb sobre su desplazamiento en el subpaso relativo
// a la superficie: como mucho μ × lo que se corrige en la normal. La corrección se reparte entre los dos nudos
// según su peso en el punto y su masa inversa, de modo que el punto se mueve justo eso (un nudo fijo no se mueve).
void RobotContact::contact(size_t ti, int id, int a, int b, double s, btSoftBody& body)
{
  const SolidContacts::Body& t = solids_.bodies()[ti];
  Frame& f = frames_[ti];
  btSoftBody::Node& na = body.m_nodes[a];
  btSoftBody::Node& nb = body.m_nodes[b];
  const double wa = 1.0 - s, wb = s;
  const double share = a == b ? na.m_im : wa * wa * na.m_im + wb * wb * nb.m_im;
  if (share <= 0.0) {
    return;  // los dos extremos fijos
  }
  const Eigen::Vector3d p = wa * eig(na.m_x) + wb * eig(nb.m_x);
  const Eigen::Vector3d* exit_local = nullptr;
  if (const auto now = f.touching.find(id); now != f.touching.end()) {
    exit_local = &now->second;
  } else if (const auto before = exits_[ti].find(id); before != exits_[ti].end()) {
    exit_local = &before->second;
  }
  const auto c =
    exit_local ? core::box_contact_along(t.box, f.pose, p, *exit_local) : core::box_contact(t.box, f.pose, p);
  if (!c) {
    return;
  }
  const Eigen::Vector3d normal_fix = c->depth * c->normal;
  // desplazamiento del punto en el subpaso respecto a la superficie del enlace, ya corregida la normal
  const Eigen::Vector3d q = wa * eig(na.m_q) + wb * eig(nb.m_q);
  const Eigen::Vector3d v_surface = f.v_cg + f.w.cross(c->surface_point - f.cg);
  const Eigen::Vector3d slip = (p + normal_fix - q) - v_surface * dt_;
  const Eigen::Vector3d slip_t = slip - slip.dot(c->normal) * c->normal;
  const double slip_norm = slip_t.norm();
  Eigen::Vector3d fix = normal_fix;
  if (slip_norm > 0.0) {
    fix -= slip_t * std::min(1.0, t.friction * c->depth / slip_norm);
  }

  Eigen::Vector3d moved_mass = Eigen::Vector3d::Zero();  // Σ m·Δx de los nudos
  auto push = [&](btSoftBody::Node& n, double weight) {
    if (n.m_im <= 0) {
      return;
    }
    const Eigen::Vector3d dx = (weight * n.m_im / share) * fix;
    n.m_x += bt(dx);
    moved_mass += dx / n.m_im;
  };
  if (a == b) {
    push(na, 1.0);
  } else {
    push(na, wa);
    push(nb, wb);
  }
  f.momentum += moved_mass;
  f.angular += (c->surface_point - f.cg).cross(moved_mass);
  f.touching[id] = f.pose.linear().transpose() * c->normal;
}

void RobotContact::end_step()
{
  // La red cambia su cantidad de movimiento en Σ m·Δx/dt: el cuerpo recibe lo contrario en el tick siguiente (una
  // sola pasada por tick: reaction_force con n = 1)
  std::vector<core::BodyReaction> reactions;
  reactions.reserve(frames_.size());
  for (size_t ti = 0; ti < frames_.size(); ++ti) {
    Frame& f = frames_[ti];
    reactions.push_back({f.momentum, f.angular});
    exits_[ti].swap(f.touching);
  }
  solids_.set_reactions(reactions, dt_, 1);
}

}  // namespace stonefish_net
