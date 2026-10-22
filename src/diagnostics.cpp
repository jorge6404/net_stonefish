#include "stonefish_net/diagnostics.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include "stonefish_net/core/net_topology.hpp"

namespace stonefish_net
{
namespace
{
const char* kind_name(core::NodeKind kind)
{
  switch (kind) {
    case core::NodeKind::Knot: return "red";
    case core::NodeKind::Dropper: return "orinque";
    case core::NodeKind::Mooring: return "fondeo";
  }
  return "?";
}
}  // namespace

void NetDiagnostics::sample(double dt, const std::vector<Eigen::Vector3f>& x, const core::NetTopology& topo,
                            double floor_z)
{
  ++step_;
  if (!reported_nonfinite_) {
    for (size_t k = 0; k < x.size(); ++k) {
      if (!x[k].allFinite()) {
        std::fprintf(stderr, "[stonefish_net] tick %ld: nudo %zu (%s) no finito: x=(%g,%g,%g)\n", step_, k,
                     kind_name(topo.nodes[k].kind), x[k].x(), x[k].y(), x[k].z());
        reported_nonfinite_ = true;
        break;
      }
    }
  }
  vmax_ = vsum_ = 0.0;
  kmax_ = 0;
  on_floor_ = 0;
  if (prev_x_.size() == x.size()) {
    const bool have_dx = prev_dx_.size() == x.size();
    prev_dx_.resize(x.size(), Eigen::Vector3f::Zero());
    node_flip_count_.resize(x.size(), 0);
    for (size_t k = 0; k < x.size(); ++k) {
      const Eigen::Vector3f d = x[k] - prev_x_[k];
      // Invierte la velocidad de un tick al siguiente moviéndose más de 5 mm/s: vibra
      if (have_dx && d.norm() > 0.005 * dt && d.dot(prev_dx_[k]) < 0.f) {
        ++node_flips_;
        if (++node_flip_count_[k] > flip_count_max_) {
          flip_count_max_ = node_flip_count_[k];
          flip_node_ = k;
        }
      }
      prev_dx_[k] = d;
      const double speed = static_cast<double>(d.norm()) / dt;
      vsum_ += speed;
      if (speed > vmax_) {
        vmax_ = speed;
        kmax_ = k;
      }
      if (x[k].z() > floor_z - 0.01) ++on_floor_;
    }
  }
  prev_x_ = x;

  const long per_second = std::lround(1.0 / dt);
  if (per_second > 0 && step_ % per_second == 0 && !x.empty()) {
    report(dt, per_second, x, topo);
  }
}

void NetDiagnostics::report(double dt, long per_second, const std::vector<Eigen::Vector3f>& x,
                            const core::NetTopology& topo)
{
  const double ticks = static_cast<double>(per_second);
  std::fprintf(stderr, "[stonefish_net] t=%g s: |v| máx %g m/s (nudo %zu, %s, z=%g), media %g m/s, en el suelo %d\n",
               step_ * dt, vmax_, kmax_, kind_name(topo.nodes[kmax_].kind), x[kmax_].z(),
               vsum_ / static_cast<double>(x.size()), on_floor_);
  if (node_flips_ > 0) {
    std::fprintf(stderr,
                 "[stonefish_net] vibración: %.1f nudos por tick invierten la velocidad; el que más, nudo %zu (%s, "
                 "z=%g) %zu veces en el último segundo\n",
                 static_cast<double>(node_flips_) / ticks, flip_node_, kind_name(topo.nodes[flip_node_].kind),
                 x[flip_node_].z(), flip_count_max_);
  } else {
    std::fprintf(stderr, "[stonefish_net] vibración: ningún nudo invierte la velocidad de un tick al siguiente\n");
  }
  node_flips_ = 0;
  flip_count_max_ = 0;
  std::fill(node_flip_count_.begin(), node_flip_count_.end(), 0u);

  if (!buoy_flips_.empty()) {
    std::string flips;
    for (int f : buoy_flips_) flips += (flips.empty() ? "" : ", ") + std::to_string(f);
    std::fprintf(stderr,
                 "[stonefish_net] boyas: |v| máx del ojal %.3f m/s, |w| máx %.2f rad/s, el ojal invierte la velocidad "
                 "[%s] veces en el último segundo\n",
                 buoy_v_max_, buoy_w_max_, flips.c_str());
    std::fill(buoy_flips_.begin(), buoy_flips_.end(), 0);
    buoy_v_max_ = buoy_w_max_ = 0.0;
  }

  if (bullet_) {
    const double ms = 1e3 / ticks;
    std::fprintf(stderr,
                 "[stonefish_net] tiempo por tick: fuerzas %.3f ms, Bullet (anclajes, contactos nativos, tramos) "
                 "%.3f ms, contacto con los sólidos %.3f ms\n",
                 bullet_->forces_s * ms, bullet_->bullet_s * ms, bullet_->contact_s * ms);
    if (bullet_->tiles > 0) {
      std::fprintf(stderr, "[stonefish_net] baldosas despiertas %d de %d, nudos calculados %zu de %zu\n",
                   bullet_->awake_tiles, bullet_->tiles, bullet_->active_nodes, topo.nodes.size());
    }
    bullet_.reset();
  }
  if (gpu_) {
    std::fprintf(stderr,
                 "[stonefish_net] GPU %.3f ms/tick; CPU mandando los subpasos %.3f ms/tick, espera y lectura %.3f "
                 "ms/tick; %d colores%s\n",
                 gpu_->gpu_ms / ticks, gpu_->issue_ms / ticks, gpu_->readback_ms / ticks, gpu_->colors,
                 gpu_->lost_contacts > 0 ? " — AVISO: contactos sin reacción (registro lleno)" : "");
    gpu_.reset();
  }
}

void NetDiagnostics::sample_buoys(const std::vector<Eigen::Vector3d>& eye_velocity, const std::vector<double>& angular_speed)
{
  prev_eye_v_.resize(eye_velocity.size(), Eigen::Vector3d::Zero());
  buoy_flips_.resize(eye_velocity.size(), 0);
  for (size_t k = 0; k < eye_velocity.size(); ++k) {
    if (eye_velocity[k].norm() > 0.005 && eye_velocity[k].dot(prev_eye_v_[k]) < 0.0) ++buoy_flips_[k];
    prev_eye_v_[k] = eye_velocity[k];
    buoy_v_max_ = std::max(buoy_v_max_, eye_velocity[k].norm());
    buoy_w_max_ = std::max(buoy_w_max_, angular_speed[k]);
  }
}

void NetDiagnostics::add_bullet(const BulletCost& c)
{
  if (!bullet_) bullet_.emplace();
  bullet_->forces_s += c.forces_s;
  bullet_->bullet_s += c.bullet_s;
  bullet_->contact_s += c.contact_s;
  bullet_->awake_tiles = c.awake_tiles;
  bullet_->tiles = c.tiles;
  bullet_->active_nodes = c.active_nodes;
}

void NetDiagnostics::add_gpu(const GpuCost& c)
{
  if (!gpu_) gpu_.emplace();
  gpu_->gpu_ms += c.gpu_ms;
  gpu_->issue_ms += c.issue_ms;
  gpu_->readback_ms += c.readback_ms;
  gpu_->colors = c.colors;
  gpu_->lost_contacts += c.lost_contacts;
}

}  // namespace stonefish_net
