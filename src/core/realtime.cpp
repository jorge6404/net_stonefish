#include "stonefish_net/core/realtime.hpp"

#include <algorithm>

namespace stonefish_net::core
{

uint64_t CatchUpClock::read(uint64_t wall_us)
{
  if (!started_) {
    started_ = true;
    last_wall_ = clock_ = wall_us;
    return clock_;
  }
  const uint64_t delta = wall_us > last_wall_ ? wall_us - last_wall_ : 0;
  last_wall_ = wall_us;
  const uint64_t step = std::min(delta, max_step_);
  dropped_ += delta - step;
  clock_ += step;
  return clock_;
}

void RealtimeReport::add_net_tick(double ms)
{
  net_sum_ms_ += ms;
  net_max_ms_ = std::max(net_max_ms_, ms);
  ++ticks_;
}

void RealtimeReport::restart(uint64_t wall_us, double sim_s, uint64_t dropped_us)
{
  wall0_ = wall_us;
  sim0_ = sim_s;
  dropped0_ = dropped_us;
  slept_ms_ = net_sum_ms_ = net_max_ms_ = 0.0;
  ticks_ = 0;
}

std::optional<RealtimeReport::Window> RealtimeReport::read(uint64_t wall_us, double sim_s, uint64_t dropped_us)
{
  const bool paused = started_ && wall_us > last_read_ + 500'000;
  last_read_ = wall_us;
  if (!started_ || paused) {
    started_ = true;
    started_us_ = wall_us;
    restart(wall_us, sim_s, dropped_us);
    return std::nullopt;
  }
  if (wall_us < wall0_ + 1'000'000) {
    return std::nullopt;
  }
  Window w;
  w.wall_s = 1e-6 * static_cast<double>(wall_us - wall0_);
  w.sim_s = sim_s - sim0_;
  w.dropped_ms = 1e-3 * static_cast<double>(dropped_us - dropped0_);
  w.ticks = ticks_;
  const double ticks = static_cast<double>(std::max(ticks_, 1L));
  w.busy_ms = std::max(1e3 * w.wall_s - slept_ms_, 0.0) / ticks;
  w.net_ms = net_sum_ms_ / ticks;
  w.net_max_ms = net_max_ms_;
  w.started_us = started_us_;
  restart(wall_us, sim_s, dropped_us);
  return w;
}

}  // namespace stonefish_net::core
