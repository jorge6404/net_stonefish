// Reloj de Stonefish con la recuperación acotada y el informe de tiempo real (NetSimulationManager::getSimulationClock).
//
// Stonefish (SimulationManager::AdvanceSimulation) pide a Bullet todo el tiempo de reloj que ha pasado desde su
// llamada anterior, sin límite de ticks. Si un tick cuesta más de lo que simula, cada llamada tiene que recuperar
// más ticks que la anterior y tarda más, y el dibujo, que solo se actualiza entre llamadas, va a saltos cada vez
// más largos. Este reloj avanza como el de verdad, pero entre dos lecturas nunca más de max_step_us: lo que no da
// tiempo a simular se pierde y la simulación va a cámara lenta, fluida.
#pragma once

#include <cstdint>
#include <optional>

namespace stonefish_net::core
{

// Ticks que Stonefish puede recuperar de golpe: a 200 Hz, 15 ms, menos que un fotograma a 60 Hz
inline constexpr int kCatchUpTicks = 3;

class CatchUpClock
{
public:
  explicit CatchUpClock(uint64_t max_step_us) : max_step_(max_step_us) {}

  // wall_us: el reloj de verdad. Si retrocede, no avanza.
  uint64_t read(uint64_t wall_us);
  // Reloj perdido en total por no recuperarlo
  uint64_t dropped_us() const { return dropped_; }

private:
  uint64_t max_step_;
  bool started_ = false;
  uint64_t last_wall_ = 0, clock_ = 0, dropped_ = 0;
};

// Tiempo real por ventanas de un segundo de reloj: lo simulado, lo perdido por no llegar, y cómo se reparte el tick
// entre la red y el resto. Se alimenta en el hilo de la física: lo que Stonefish duerme (add_sleep), lo que cuesta
// la red en cada tick (add_net_tick) y cada lectura del reloj (read).
class RealtimeReport
{
public:
  struct Window
  {
    double wall_s, sim_s, dropped_ms;
    long ticks;
    double busy_ms;               // por tick, lo no dormido: la red, Bullet, Stonefish (sensores, dibujo) y ROS
    double net_ms, net_max_ms;    // por tick, la red; y su tick más caro
    uint64_t started_us;          // reloj en que empezó a medir tras una pausa (o al arrancar)
  };

  void add_sleep(double ms) { slept_ms_ += ms; }
  void add_net_tick(double ms);

  // wall_us: el reloj de verdad; sim_s: tiempo simulado; dropped_us: CatchUpClock::dropped_us. Medio segundo sin
  // leer el reloj es una pausa (o la carga de los shaders en el primer tick) y vuelve a empezar sin contarla.
  // Devuelve la ventana cuando ha pasado un segundo.
  std::optional<Window> read(uint64_t wall_us, double sim_s, uint64_t dropped_us);

private:
  void restart(uint64_t wall_us, double sim_s, uint64_t dropped_us);

  bool started_ = false;
  uint64_t last_read_ = 0, wall0_ = 0, dropped0_ = 0, started_us_ = 0;
  double sim0_ = 0.0, slept_ms_ = 0.0, net_sum_ms_ = 0.0, net_max_ms_ = 0.0;
  long ticks_ = 0;
};

}  // namespace stonefish_net::core
