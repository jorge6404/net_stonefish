// Diagnóstico opcional de la red (diagnostics.enabled en el YAML): aviso del primer nudo no finito y un
// resumen por segundo simulado en stderr (velocidades, vibración, nudos en el suelo, boyas y el coste del solver).
// Lo posee el solver (BulletSolver o GpuSolver), que lo alimenta una vez por tick con las posiciones de la red y la
// velocidad de las boyas; cada solver añade su coste (BulletCost, GpuCost). Nace y muere con el escenario.
#pragma once

#include <optional>
#include <vector>

#include <Eigen/Core>

namespace stonefish_net
{
namespace core
{
struct NetTopology;
}

class NetDiagnostics
{
public:
  // Una vez por tick, con las posiciones del final. Las velocidades salen de las posiciones de dos ticks.
  void sample(double dt, const std::vector<Eigen::Vector3f>& x, const core::NetTopology& topo, double floor_z);

  // Una vez por tick: velocidad del ojal y velocidad angular de cada boya (|v| y |w| máximos y cuántas veces por
  // segundo invierte el ojal su velocidad: una boya que vibra)
  void sample_buoys(const std::vector<Eigen::Vector3d>& eye_velocity, const std::vector<double>& angular_speed);

  // BulletSolver, por tick: segundos de reloj en cada parte y las baldosas dormidas
  struct BulletCost
  {
    double forces_s = 0.0, bullet_s = 0.0, contact_s = 0.0;
    int awake_tiles = 0, tiles = 0;
    size_t active_nodes = 0;
  };
  void add_bullet(const BulletCost& c);

  // GpuSolver, por tick: tiempo de GPU, de la CPU mandando los subpasos y de espera + lectura (ms), colores y
  // contactos sin reacción
  struct GpuCost
  {
    double gpu_ms = 0.0, issue_ms = 0.0, readback_ms = 0.0;
    int colors = 0;
    unsigned long lost_contacts = 0;
  };
  void add_gpu(const GpuCost& c);

private:
  void report(double dt, long per_second, const std::vector<Eigen::Vector3f>& x, const core::NetTopology& topo);

  long step_ = 0;
  bool reported_nonfinite_ = false;
  // Velocidades del último tick
  double vmax_ = 0.0, vsum_ = 0.0;
  size_t kmax_ = 0;
  int on_floor_ = 0;
  std::vector<Eigen::Vector3f> prev_x_, prev_dx_;
  long node_flips_ = 0;  // nudos que se mueven y han invertido la velocidad respecto al tick anterior
  size_t flip_node_ = 0, flip_count_max_ = 0;
  std::vector<unsigned> node_flip_count_;
  // Boyas
  std::vector<Eigen::Vector3d> prev_eye_v_;
  std::vector<int> buoy_flips_;
  double buoy_v_max_ = 0.0, buoy_w_max_ = 0.0;
  // Coste del solver en el último segundo
  std::optional<BulletCost> bullet_;
  std::optional<GpuCost> gpu_;
};

}  // namespace stonefish_net
