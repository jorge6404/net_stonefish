// Parámetros de la red (config/net.yaml). Sin ROS ni Stonefish.
#pragma once

#include <string>
#include <vector>

#include "stonefish_net/core/contact.hpp"

namespace stonefish_net::core
{

struct NetConfig
{
  struct Net { double width, height, bar, opening_deg, twine_diameter, twine_density; } net;
  struct Pose { double x, y_center, z_top; } pose;
  struct FrameRope { double diameter, density; } frame_rope;
  struct Leads { double spacing, mass; } leads;
  struct Buoys { double spacing, dropper, mass; } buoys;  // tamaño: el del asset; mass: boya hueca real (kg)
  // Una línea de fondeo: ancla desplazada `outward` hacia fuera en y, longitud = distancia × slack.
  struct Line { double outward, slack; };
  // bottom: esquinas inferiores (con holgura); top: superiores (tensas, mantienen la red abierta a lo ancho)
  struct Mooring { double floor_z, upstream, block_height; Line bottom, top; } mooring;
  // current_grid: lado de la rejilla en la que el solver de GPU muestrea la corriente del océano, m
  struct Hydro { double cn, ct, added_mass, water_density, current_grid; } hydro;
  // contacto del cuerpo blando: kCHR, margen de colisión (m), kDP. friction: μ con el suelo (floor_force) y kDF
  // de Bullet; contra los sólidos de Stonefish no cuenta (Bullet usa kDF × rozamiento del rígido, que Stonefish
  // deja a 0): con las boyas no hay rozamiento y con los robots lo pone robot_contact.
  // backend: gpu (XPBD en compute shaders) o bullet (cuerpo blando de Bullet). iterations y contact_hardness
  // solo cuentan con bullet; substeps (subpasos XPBD por tick) y compliance (flexibilidad del hilo, m/N; 0 =
  // inextensible) solo con gpu.
  enum class Backend { Gpu, Bullet };
  struct Solver
  {
    Backend backend;
    int iterations, substeps;
    double contact_hardness, margin, damping, friction, compliance;
  } solver;
  // Agujeros aleatorios reproducibles: rotos (grupos de rombos) y hilos sueltos cortados
  struct Damage { int seed, tears, tear_min_meshes, tear_max_meshes; double stub_fraction, loose_fraction; } damage;
  // Enlaces de robot con contacto propio (RobotContact) en vez del de Bullet, que con SDF_RS solo mira los
  // nudos y deja pasar el robot entre ellos. Enlace de Stonefish: "<robot>/<link>".
  // Un enlace que no exista en el escenario se ignora con un aviso.
  struct RobotLink { std::string robot, link; RoundedBox box; double friction; };
  std::vector<RobotLink> robot_contact;
  // Contacto propio también para los enlaces de robot que no están en robot_contact: caja envolvente de su
  // forma de colisión, con radio de redondeo = rounding_fraction × su semieje menor.
  struct RobotContactAuto { bool enabled; double rounding_fraction, friction; } robot_contact_auto;
  // Lo mismo para los sólidos dinámicos que no son enlaces de robot (boyas, objetos sueltos del escenario)
  RobotContactAuto solid_contact_auto;
  // Baldosas dormidas (core/tiles.hpp): la red se parte en cuadrados de tile_size m y uno se congela cuando lleva
  // settle_time s con todos sus nudos por debajo de sleep_speed (m/s) y a más de sleep_radius m de los robots. Se
  // despierta a menos de wake_radius m de un robot o si el nudo de fuera de un tramo de su borde se ha movido más
  // de wake_shift m desde que se durmió.
  struct Sleep { bool enabled; double tile_size, wake_radius, sleep_radius, sleep_speed, settle_time, wake_shift; } sleep;
  // Pinzas que agarran la red (solo backend gpu; core/grasp.hpp). fingers: enlaces del robot, como en robot_contact
  // («<robot>/<dedo>»), al menos dos y sin repetir. Un nudo a menos de grip_margin m de las cajas de dos dedos se
  // ata al primero de ellos hasta que la distancia entre sus centros crece más de release_gap m.
  struct Gripper { std::string robot; std::vector<std::string> fingers; double grip_margin, release_gap; };
  std::vector<Gripper> grippers;  // opcional en el YAML: si falta, ninguna
  // Resumen periódico en stderr (velocidades, inversiones, suelo, boyas) y aviso de nudos no finitos
  struct Diagnostics { bool enabled; } diagnostics;
};

// Lanzan std::invalid_argument("<seccion>.<campo>: <motivo>"). Solo validan rangos de cada campo;
// las reglas geométricas (p. ej. que el ancla quede por debajo de la red) las comprueba build_topology.
NetConfig parse_net_config(const std::string& yaml_text);
NetConfig load_net_config(const std::string& path);

}  // namespace stonefish_net::core
