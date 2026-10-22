#include "stonefish_net/core/xpbd_ref.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include <Eigen/Geometry>

#include "stonefish_net/core/hydro.hpp"

namespace stonefish_net::core
{
namespace
{
Eigen::Vector3f node_x(const GpuLayout& L, size_t k) { return {L.x0[4 * k], L.x0[4 * k + 1], L.x0[4 * k + 2]}; }
float node_w(const GpuLayout& L, size_t k) { return L.x0[4 * k + 3]; }

constexpr float kEnd = 1e-3f;  // punto más hundido en un extremo del tramo: ya lo trata el nudo

// Un cuerpo en el subpaso en curso
struct BodyNow
{
  Eigen::Isometry3d pose;
  Eigen::Vector3d cg;
};

const Eigen::Vector4f* find_slot(const Eigen::Vector4f* slots, int body)
{
  for (int j = 0; j < kMaxContactBodies; ++j) {
    if (static_cast<int>(slots[j].w()) == body) return &slots[j];
  }
  return nullptr;
}

// Saca de la caja del cuerpo b el punto (1 − u)·x_a + u·x_b (un nudo: a == b), igual que RobotContact::contact:
// por la dirección recordada si ya estaba en contacto, si no por la superficie más cercana; Coulomb sobre su
// desplazamiento en el subpaso relativo a la superficie; la corrección se reparte según peso y masa inversa.
// Devuelve la dirección de salida (marco de la caja) si hubo contacto, para la memoria del subpaso siguiente.
bool contact_point(const GpuLayout& L, const XpbdParams& p, const ContactBody& body, const BodyNow& now, int b,
                   const Eigen::Vector4f* old_slots, size_t a, size_t c, float u, XpbdState& s, BodyReaction& reaction,
                   Eigen::Vector3f& exit_out)
{
  const float wa = 1.f - u, wb = u;
  const float ima = node_w(L, a), imb = node_w(L, c);
  const float share = a == c ? ima : wa * wa * ima + wb * wb * imb;
  if (share <= 0.f) return false;  // los dos extremos fijos
  const Eigen::Vector3d pt = (wa * s.x[a] + wb * s.x[c]).cast<double>();
  std::optional<Contact> hit;
  if (const Eigen::Vector4f* slot = find_slot(old_slots, b)) {
    hit = box_contact_along(body.box, now.pose, pt, slot->head<3>().cast<double>().normalized());
  } else {
    hit = box_contact(body.box, now.pose, pt);
  }
  if (!hit) return false;
  const Eigen::Vector3d normal_fix = hit->depth * hit->normal;
  const Eigen::Vector3d disp = (wa * s.dx[a] + wb * s.dx[c]).cast<double>();  // desplazamiento del punto en el subpaso
  const Eigen::Vector3d v_surface = body.v_cg + body.w.cross(hit->surface_point - now.cg);
  const Eigen::Vector3d slip = disp + normal_fix - v_surface * p.h;
  const Eigen::Vector3d slip_t = slip - slip.dot(hit->normal) * hit->normal;
  const double slip_norm = slip_t.norm();
  Eigen::Vector3d fix = normal_fix;
  if (slip_norm > 0.0) fix -= slip_t * std::min(1.0, body.friction * hit->depth / slip_norm);

  Eigen::Vector3d moved = Eigen::Vector3d::Zero();  // Σ m·Δx de los nudos
  auto push = [&](size_t k, float weight, float im) {
    if (im <= 0.f) return;
    const Eigen::Vector3f d = ((weight * im / share) * fix).cast<float>();
    s.x[k] += d;
    s.dx[k] += d;
    moved += d.cast<double>() / im;
  };
  if (a == c) {
    push(a, 1.f, ima);
  } else {
    push(a, wa, ima);
    push(c, wb, imb);
  }
  reaction.momentum += moved;
  reaction.angular += (hit->surface_point - now.cg).cross(moved);
  exit_out = (now.pose.linear().transpose() * hit->normal).cast<float>();
  return true;
}

// Guarda en new_slots la salida de un contacto con el cuerpo b (si queda hueco)
void remember(std::array<Eigen::Vector4f, kMaxContactBodies>& new_slots, int b, const Eigen::Vector3f& exit)
{
  for (auto& slot : new_slots) {
    if (slot.w() < 0.f) {
      slot << exit, static_cast<float>(b);
      return;
    }
  }
}
}  // namespace

ContactMemory contact_memory(const GpuLayout& L)
{
  const Eigen::Vector4f free(0.f, 0.f, 0.f, -1.f);
  return {std::vector<Eigen::Vector4f>(kMaxContactBodies * L.node_count(), free),
          std::vector<Eigen::Vector4f>(kMaxContactBodies * L.link_count(), free)};
}

XpbdState xpbd_init(const GpuLayout& L)
{
  XpbdState s;
  const size_t n = L.node_count();
  s.v.assign(n, Eigen::Vector3f::Zero());
  for (size_t k = 0; k < n; ++k) s.x.push_back(node_x(L, k));
  s.dx.assign(n, Eigen::Vector3f::Zero());
  return s;
}

void xpbd_substep(const GpuLayout& L, const XpbdParams& p, const std::function<Eigen::Vector3f(const Eigen::Vector3f&)>& water,
                  const std::vector<Anchor>& anchors, const std::vector<ContactBody>& bodies, int i, int n, XpbdState& s,
                  ContactMemory& memory, std::vector<Eigen::Vector3f>& anchor_dx, std::vector<BodyReaction>& reactions)
{
  const size_t nodes = L.node_count();

  // a. Fuerzas con el estado del subpaso anterior (todas antes de mover ningún nudo) y predicción
  std::vector<Eigen::Vector3f> v_new(nodes);
  for (size_t k = 0; k < nodes; ++k) {
    const float w = node_w(L, k);
    if (w <= 0.f) {
      v_new[k] = Eigen::Vector3f::Zero();
      continue;
    }
    const Eigen::Vector3f wk = water(s.x[k]);
    Eigen::Vector3f drag = Eigen::Vector3f::Zero();
    for (int a = L.adj_begin[k]; a < L.adj_begin[k + 1]; ++a) {
      const auto i = static_cast<size_t>(L.adj_links[static_cast<size_t>(a)]);
      const auto na = static_cast<size_t>(L.links[2 * i]), nb = static_cast<size_t>(L.links[2 * i + 1]);
      drag += 0.5f * morison_force<float>(s.x[na], s.x[nb], s.v[na], s.v[nb], 0.5f * (water(s.x[na]) + water(s.x[nb])),
                                          L.link_props[4 * i + 1], p.rho, p.cn, p.ct);
    }
    const float mass = L.node_props[4 * k], volume = L.node_props[4 * k + 1], inertial = L.node_props[4 * k + 2];
    const Eigen::Vector3f f = limit_drag<float>(drag, wk - s.v[k], inertial, p.h) +
                              Eigen::Vector3f(0.f, 0.f, (mass - p.rho * volume) * p.g);
    v_new[k] = s.v[k] + p.h * w * f;
  }
  for (size_t k = 0; k < nodes; ++k) {
    s.v[k] = v_new[k];
    s.dx[k] = p.h * s.v[k];
    s.x[k] += s.dx[k];
  }

  // b. Tramos, color a color (dentro de un color ningún nudo se repite: el orden no cambia el resultado)
  const float alpha = p.compliance / (p.h * p.h);
  for (size_t i = 0; i < L.link_count(); ++i) {
    const auto a = static_cast<size_t>(L.links[2 * i]), b = static_cast<size_t>(L.links[2 * i + 1]);
    const Eigen::Vector3f d = s.x[b] - s.x[a];
    const float len = d.norm();
    const float c = len - L.link_props[4 * i];
    const float wa = node_w(L, a), wb = node_w(L, b);
    if (len < 1e-9f || c <= 0.f || wa + wb + alpha <= 0.f) {
      continue;  // degenerado, flojo (un hilo no empuja) o con los dos extremos fijos
    }
    const Eigen::Vector3f dir = d / len;
    const float dl = -c / (wa + wb + alpha);
    const Eigen::Vector3f fix_a = -wa * dl * dir, fix_b = wb * dl * dir;
    s.x[a] += fix_a;
    s.dx[a] += fix_a;
    s.x[b] += fix_b;
    s.dx[b] += fix_b;
  }

  // c. Anclajes
  anchor_dx.resize(anchors.size(), Eigen::Vector3f::Zero());
  if (i == 0) {
    s.anchor_offset.assign(anchors.size(), Eigen::Vector3f::Zero());
    s.anchor_vel.assign(anchors.size(), Eigen::Vector3f::Zero());
  }
  for (size_t j = 0; j < anchors.size(); ++j) {
    const auto k = static_cast<size_t>(anchors[j].node);
    const float w = node_w(L, k);
    if (w <= 0.f) continue;
    // El ojal, en su camino sin la red más lo que la red lo ha apartado (con su inercia)
    const Eigen::Vector3f offset = s.anchor_offset[j] + p.h * s.anchor_vel[j];
    const Eigen::Vector3f eye =
        anchors[j].from + (static_cast<float>(i + 1) / n) * (anchors[j].to - anchors[j].from) + offset;
    // C = x − ojal; lambda = (w·I + inv_mass)⁻¹ C: el nudo se mueve −w·lambda y el ojal inv_mass·lambda
    const Eigen::Vector3f lambda =
        (w * Eigen::Matrix3f::Identity() + anchors[j].inv_mass).inverse() * (s.x[k] - eye);
    const Eigen::Vector3f dx = -w * lambda;
    s.x[k] += dx;
    s.dx[k] += dx;
    anchor_dx[j] += dx / w;
    const Eigen::Vector3f moved = offset + anchors[j].inv_mass * lambda;
    s.anchor_vel[j] = (moved - s.anchor_offset[j]) / p.h;
    s.anchor_offset[j] = moved;
  }

  // c'. Contacto con los sólidos en su pose de este subpaso: nudos, luego tramos color a color
  reactions.resize(bodies.size(), BodyReaction{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()});
  const double frac = static_cast<double>(i + 1) / n;
  std::vector<BodyNow> now;
  const std::vector<uint32_t> excluded = exclusion_masks(bodies, L.node_count());
  const auto ignores = [&excluded](size_t k, size_t b) { return excludes(excluded[k], b); };
  for (size_t b = 0; b < bodies.size(); ++b) {
    now.push_back({interpolate_pose(bodies[b].pose0, bodies[b].pose1, frac),
                   (1.0 - frac) * bodies[b].cg0 + frac * bodies[b].cg1});
  }
  const Eigen::Vector4f free(0.f, 0.f, 0.f, -1.f);
  std::array<Eigen::Vector4f, kMaxContactBodies> new_slots;
  Eigen::Vector3f exit;
  for (size_t k = 0; k < nodes; ++k) {
    Eigen::Vector4f* slots = &memory.node[kMaxContactBodies * k];
    new_slots.fill(free);
    for (size_t b = 0; b < bodies.size(); ++b) {
      if (ignores(k, b) ||
          !point_in_box_bounds(bodies[b].box, now[b].pose, s.x[k].cast<double>())) {
        continue;
      }
      if (contact_point(L, p, bodies[b], now[b], static_cast<int>(b), slots, k, k, 0.f, s, reactions[b], exit)) {
        remember(new_slots, static_cast<int>(b), exit);
      }
    }
    std::copy(new_slots.begin(), new_slots.end(), slots);
  }
  for (size_t li = 0; li < L.link_count(); ++li) {
    const auto a = static_cast<size_t>(L.links[2 * li]), c = static_cast<size_t>(L.links[2 * li + 1]);
    Eigen::Vector4f* slots = &memory.link[kMaxContactBodies * li];
    new_slots.fill(free);
    for (size_t b = 0; b < bodies.size(); ++b) {
      const int bi = static_cast<int>(b);
      if (ignores(a, b) || ignores(c, b)) continue;
      const Eigen::Vector3d xa = s.x[a].cast<double>(), xc = s.x[c].cast<double>();
      if (!segment_in_box_bounds(bodies[b].box, now[b].pose, xa, xc)) continue;
      const auto u = static_cast<float>(deepest_on_segment(bodies[b].box, now[b].pose, xa, xc));
      if (u <= kEnd || u >= 1.f - kEnd) continue;
      if (contact_point(L, p, bodies[b], now[b], bi, slots, a, c, u, s, reactions[b], exit)) {
        remember(new_slots, bi, exit);
      }
    }
    std::copy(new_slots.begin(), new_slots.end(), slots);
  }

  // d. Suelo, y e. velocidades
  for (size_t k = 0; k < nodes; ++k) {
    if (node_w(L, k) <= 0.f) {
      s.v[k] = Eigen::Vector3f::Zero();
      continue;
    }
    Eigen::Vector3f& x = s.x[k];
    if (x.z() > p.floor_z) {
      const float depth = x.z() - p.floor_z;
      x.z() = p.floor_z;
      s.dx[k].z() -= depth;
      // Coulomb sobre el desplazamiento horizontal del subpaso: como mucho μ × lo corregido en la normal
      const Eigen::Vector2f slip = s.dx[k].head<2>();
      const float limit = p.friction * depth, norm = slip.norm();
      const Eigen::Vector2f fix = norm <= limit ? slip : Eigen::Vector2f(slip * (limit / norm));
      x.head<2>() -= fix;
      s.dx[k].head<2>() -= fix;
    }
    s.v[k] = s.dx[k] / p.h * (1.f - p.damping);
  }
}

}  // namespace stonefish_net::core
