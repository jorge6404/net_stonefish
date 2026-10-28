// Escenas de prueba de la red, comunes a los tests (test/*.cpp) y a tools/gpu_check: la red de test/net_test.yaml,
// ticks de Stonefish a 200 Hz, cajas que se mueven, cadenas colgadas y la escena de la pinza. Sin GPU.
#pragma once

#include <array>
#include <cmath>
#include <functional>
#include <set>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "stonefish_net/core/coloring.hpp"
#include "stonefish_net/core/gpu_layout.hpp"
#include "stonefish_net/core/grasp.hpp"
#include "stonefish_net/core/net_config.hpp"
#include "stonefish_net/core/net_topology.hpp"
#include "stonefish_net/core/xpbd_ref.hpp"

namespace stonefish_net::test
{

namespace core = stonefish_net::core;

constexpr double kTick = 0.005;  // s, un tick de Stonefish a 200 Hz
inline int ticks_in(double seconds) { return static_cast<int>(std::lround(seconds / kTick)); }
inline const Eigen::Vector3f kCurrent(0.2f, 0.f, 0.f);  // m/s, la del escenario del tanque

// test/net_test.yaml (NET_TEST_CONFIG), cargada y su topología
const std::string& config_path();
const core::NetConfig& config();
const core::NetTopology& topology();

// Parámetros de un subpaso con la configuración del YAML; dry: sin arrastre
core::XpbdParams params(const core::NetConfig& cfg);
core::XpbdParams dry(const core::NetConfig& cfg);
// El agua de las pruebas: kCurrent en todas partes
Eigen::Vector3f current_at(const Eigen::Vector3f&);

// Las boyas, quietas donde empiezan: el extremo de cada orinque anclado a su posición inicial
std::vector<core::Anchor> fixed_buoys(const core::NetTopology& topo);
// Los nudos de esos anclajes
std::vector<int> buoy_nodes(const core::NetTopology& topo);
// Centro de la red en su posición inicial, a media altura
Eigen::Vector3d net_middle(const core::NetConfig& cfg);
Eigen::Vector3f center(const std::vector<Eigen::Vector3f>& x);
// Lo que más se estira un tramo: (longitud − natural)/natural
float max_stretch(const core::NetTopology& topo, const std::vector<Eigen::Vector3f>& x);
float relative(const Eigen::Vector3d& got, const Eigen::Vector3d& want);

// Cadena vertical de `count` nudos de 10 g (20 g de masa inercial, sin volumen) separados 10 cm, colgada de su
// primer nudo, sin agua: lo más sencillo con un peso conocido
core::GpuLayout hanging_chain(int count);

// Caja de 0,6 × 0,5 × 0,3 m con el giro `rot` que va de c0 a c1 en un tick
core::ContactBody moving_box(const Eigen::Vector3d& c0, const Eigen::Vector3d& c1,
                             const Eigen::Matrix3d& rot = Eigen::Matrix3d::Identity());

// Tres cajas quietas de 1 cm que contienen el nudo libre más cercano al centro de la red (y ningún otro); las dos
// primeras lo excluyen y la tercera no
struct ExclusionCase
{
  std::vector<core::ContactBody> bodies;
  int node;
};
ExclusionCase exclusion_case(const core::NetConfig& cfg, const core::GpuLayout& L);

// Dedos de las pruebas de agarre: cajas de 1 × 4 × 4 cm (finas en x), sin giro; una pinza con los cuerpos 0 y 1
inline const core::RoundedBox kFingerBox{Eigen::Vector3d(0.005, 0.02, 0.02), 0.002, Eigen::Vector3d::Zero()};
core::Gripper gripper(double grip_margin = 0.004, double release_gap = 0.005);
Eigen::Isometry3d at(const Eigen::Vector3d& c);
// Dos dedos a ±half_gap en x de c
std::vector<core::Finger> finger_pair(const Eigen::Vector3d& c, double half_gap);

// Un tick de la red (referencia o GPU) con esos anclajes y cuerpos: deja las posiciones del final en x y lo
// acumulado por cada anclaje en anchor_dx
using NetTick = std::function<void(const std::vector<core::Anchor>&, const std::vector<core::ContactBody>&,
                                   std::vector<Eigen::Vector3f>& x, std::vector<Eigen::Vector3f>& anchor_dx)>;
NetTick ref_tick(const core::NetConfig& cfg, const core::GpuLayout& L);

// La red asentada con las boyas fijas y dos dedos (kFingerBox) a ±3 cm en x de `pinch` que se cierran a 0,02 m/s cada
// uno hasta dejar 2 mm entre caras, tiran 0,2 m en −x a 0,1 m/s, se paran 0,2 s, se abren 2 cm cada uno a 0,02 m/s y
// se alejan 0,1 m a 0,1 m/s, el 0 en −x y el 1 en +x. Cada tick, como GpuSolver: el agarre en el pre-tick con las
// posiciones del anterior, los dedos como cuerpos de contacto que ignoran lo agarrado y los anclajes de las boyas
// seguidos de los del agarre.
struct GripScene
{
  enum Phase { Close, Pull, Still, Open, Away };
  static constexpr double kClose = 1.2, kPull = 2.0, kStill = 0.2, kOpen = 1.0, kAway = 1.0;  // s

  const core::NetTopology& topo;
  NetTick tick;
  core::Grasp grasp;
  std::vector<core::Anchor> buoys;
  std::vector<Eigen::Vector3f> x, anchor_dx;
  Eigen::Vector3d pinch = Eigen::Vector3d::Zero();
  // Lo que se mide
  float settled_stretch = 0.f, stretch = 0.f, attach = 0.f;  // estiramiento al asentarse y máx; distancia al punto atado
  size_t held_close = 0;                                      // nudos agarrados al acabar de cerrar
  size_t held_pull = 0;                                       // y al acabar de tirar
  int slipped = 0;                                            // nudos soltados porque el tirón pasa de max_pull
  double max_pull_seen = 0.0;                                 // el mayor tirón de la pinza en un tick, N
  std::set<int> ever_held;
  std::vector<Eigen::Vector3f> away_start;
  Eigen::Vector3d grip_dx = Eigen::Vector3d::Zero();  // Σ de lo acumulado por los anclajes del agarre
  std::vector<int> first_held;                        // los nudos del primer tick en que agarra

  // g: la pinza (cuerpos 0 y 1); substeps: los del tick de f, para el tirón (reaction_force) que mira max_pull
  GripScene(const core::NetTopology& t, NetTick f, const core::Gripper& g = gripper(), int substeps = 20);
  int substeps;

  void settle(double seconds);
  Eigen::Vector3d nearest_node(const Eigen::Vector3d& target) const;  // el nudo libre más cercano
  static int ticks();
  static Phase phase(int t);
  std::array<Eigen::Vector3d, 2> fingers(int t) const;  // centros de los dedos al final del tick t (−1: al empezar)
  void step(int t);
  bool finite() const;
  float followed() const;  // lo más que se ha ido en −x en la fase Away un nudo que estuvo agarrado
};

}  // namespace stonefish_net::test
