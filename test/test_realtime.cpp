// Reloj de Stonefish con la recuperación acotada (core::CatchUpClock) y el informe de tiempo real
// (core::RealtimeReport)
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "stonefish_net/core/realtime.hpp"

namespace stonefish_net::test
{
namespace core = stonefish_net::core;
namespace
{
// AdvanceSimulation de Stonefish con Bullet (stepSimulation sin límite de pasos) durante `calls` llamadas, con ticks
// de dt_us que cuestan cost_us de reloj, leyendo el reloj con `read`. Devuelve el máximo de ticks en una llamada y
// los segundos simulados y de reloj.
struct LoopRun
{
  int max_ticks = 0;
  double sim_s = 0.0, wall_s = 0.0;
};
template <typename Read>
LoopRun stonefish_loop(Read read, uint64_t dt_us, uint64_t cost_us, int calls)
{
  LoopRun run;
  uint64_t wall = 1'000'000, current = read(wall), local = 0, ticks = 0;
  const uint64_t start = wall;
  for (int c = 0; c < calls; ++c) {
    uint64_t now = read(wall), delta = now - current;
    current = now;
    if (delta < dt_us) {  // SimulationClockSleep hasta completar un tick
      wall += dt_us - delta;
      now = read(wall);
      delta += now - current;
      current = now;
    }
    local += delta;
    const uint64_t n = local / dt_us;
    local -= n * dt_us;
    ticks += n;
    wall += n * cost_us + 100;  // los ticks y el resto de la vuelta
    run.max_ticks = std::max(run.max_ticks, static_cast<int>(n));
  }
  run.sim_s = 1e-6 * static_cast<double>(ticks * dt_us);
  run.wall_s = 1e-6 * static_cast<double>(wall - start);
  return run;
}
}  // namespace

// Con ticks que cuestan más de lo que simulan, Stonefish con el reloj de verdad recupera cada vez más ticks por
// llamada (el dibujo va a saltos cada vez mayores); con core::CatchUpClock, como mucho kCatchUpTicks + 1 y a cámara
// lenta. Con ticks baratos, tiempo real sin perder nada.
TEST(Realtime, CatchUpClock)
{
  constexpr uint64_t dt = 5000;  // 200 Hz
  const LoopRun spiral = stonefish_loop([](uint64_t w) { return w; }, dt, 9000, 40);
  core::CatchUpClock slow_clock(core::kCatchUpTicks * dt), fast_clock(core::kCatchUpTicks * dt);
  const LoopRun slow = stonefish_loop([&](uint64_t w) { return slow_clock.read(w); }, dt, 9000, 2000);
  const LoopRun fast = stonefish_loop([&](uint64_t w) { return fast_clock.read(w); }, dt, 2000, 2000);
  EXPECT_GE(spiral.max_ticks, 100) << "la prueba no reproduce la espiral de Stonefish";
  EXPECT_LE(slow.max_ticks, core::kCatchUpTicks + 1) << "recupera demasiados ticks de golpe";
  EXPECT_NEAR(slow.sim_s / slow.wall_s, 5.0 / 9.0, 0.02) << "no va a cámara lenta";
  EXPECT_LE(fast.max_ticks, 1);
  EXPECT_EQ(fast_clock.dropped_us(), 0u);
  EXPECT_NEAR(fast.sim_s / fast.wall_s, 1.0, 0.03) << "con ticks baratos no va en tiempo real";
}

// Ticks de 5 ms que cuestan 3 ms de red y 1 de sueño, leyendo el reloj en cada uno: cada segundo una ventana con
// esas cuentas; tras una pausa vuelve a empezar sin contarla
TEST(Realtime, Report)
{
  core::RealtimeReport report;
  uint64_t wall = 10'000'000;
  double sim = 0.0;
  int windows = 0;
  core::RealtimeReport::Window last{};
  const auto run = [&](int ticks) {
    for (int t = 0; t < ticks; ++t) {
      if (const auto w = report.read(wall, sim, 0)) {
        last = *w;
        ++windows;
      }
      report.add_net_tick(3.0);
      report.add_sleep(1.0);
      wall += 5000;
      sim += 0.005;
    }
  };
  EXPECT_FALSE(report.read(wall, sim, 0)) << "la primera lectura solo empieza a medir";
  run(401);
  ASSERT_EQ(windows, 2);
  EXPECT_EQ(last.ticks, 200);
  EXPECT_NEAR(last.wall_s, 1.0, 1e-9);
  EXPECT_NEAR(last.sim_s, 1.0, 1e-9);
  EXPECT_NEAR(last.net_ms, 3.0, 1e-9);
  EXPECT_NEAR(last.busy_ms, 4.0, 1e-9);  // 5 ms de reloj menos 1 dormido
  const uint64_t paused = wall + 2'000'000;
  wall = paused;
  run(199);
  EXPECT_EQ(windows, 2) << "cuenta la pausa en una ventana";
  run(2);
  ASSERT_EQ(windows, 3);
  EXPECT_EQ(last.started_us, paused);
  EXPECT_NEAR(last.wall_s, 1.0, 1e-9);
}

}  // namespace stonefish_net::test
