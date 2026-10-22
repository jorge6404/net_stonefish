// Baldosas dormidas de la red (sección sleep, regla en core/tiles.hpp) sobre el cuerpo blando de Bullet.
//
// Dormir una baldosa es poner a 0 la masa inversa y la velocidad de sus nudos: Bullet los trata como fijos (no
// los integra y los tramos no los mueven) y sus tramos frontera sujetan a la parte despierta como un anclaje
// (con updateLinkConstants, un tramo con los dos extremos dormidos queda fuera del bucle de tramos).
// NetForces y RobotContact solo recorren los nudos y tramos activos. Bullet sigue pasando por todos en sus
// bucles internos, pero con masa inversa 0 apenas cuestan.
#pragma once

#include <utility>
#include <vector>

#include <Eigen/Core>

#include "stonefish_net/RobotContact.h"
#include "stonefish_net/core/net_config.hpp"
#include "stonefish_net/core/net_topology.hpp"
#include "stonefish_net/core/tiles.hpp"

class btSoftBody;

namespace stonefish_net
{

class NetSleep
{
public:
  // Empieza con todo despierto: la red tiene que asentarse antes de dormir nada.
  NetSleep(const core::NetTopology& topo, const core::NetConfig::Sleep& cfg);

  // Pre-tick, antes de las fuerzas: estado de cada baldosa, y las duerme o despierta. Sin efecto con
  // sleep.enabled a false.
  void update(btSoftBody& body, double dt, const std::vector<RobotContact::Sphere>& robots);

  const std::vector<int>& active_nodes() const { return active_nodes_; }  // los que no duermen (fijos incluidos)
  const std::vector<int>& active_links() const { return active_links_; }  // tramos con algún extremo activo
  int awake_tiles() const { return awake_; }
  int tile_count() const { return static_cast<int>(tiles_.tiles.size()); }

private:
  // Por baldosa dormida: esfera que la contiene y, por tramo frontera, su nudo de fuera y dónde estaba al dormirse
  struct Frozen
  {
    Eigen::Vector3d center;
    double radius = 0;
    std::vector<std::pair<int, Eigen::Vector3d>> outside;
  };

  double gap(const Eigen::Vector3d& center, double radius, const std::vector<RobotContact::Sphere>& robots) const;
  void freeze(btSoftBody& body, size_t t);
  void wake(btSoftBody& body, size_t t);
  void rebuild();

  core::NetConfig::Sleep cfg_;
  core::NetTiles tiles_;
  std::vector<std::pair<int, int>> links_;
  std::vector<core::TileState> state_;
  std::vector<Frozen> frozen_;
  std::vector<double> im_;  // masa inversa de cada nudo antes de dormirse
  std::vector<int> active_nodes_, active_links_;
  int awake_ = 0;
};

}  // namespace stonefish_net
