// Baldosas de la red (sección sleep): los nudos de la retícula se reparten en celdas cuadradas por su posición
// inicial en el plano de la red (y, z), y cada celda se duerme entera cuando está quieta y lejos de los robots.
// Aquí solo la partición y la regla de dormir y despertar, sin Bullet; NetSleep la aplica al cuerpo blando.
#pragma once

#include <vector>

#include "stonefish_net/core/net_config.hpp"
#include "stonefish_net/core/net_topology.hpp"

namespace stonefish_net::core
{

struct Tile
{
  std::vector<int> nodes;     // nudos de la retícula (Knot, no fijos) de la celda
  std::vector<int> boundary;  // tramos con un extremo en la celda y el otro fuera
};

struct NetTiles
{
  std::vector<Tile> tiles;   // solo celdas con algún nudo
  std::vector<int> tile_of;  // por nudo: su baldosa, o -1 (orinques, fondeo y nudos fijos: nunca duermen)
};

NetTiles partition_tiles(const NetTopology& topo, double tile_size);

struct TileState
{
  bool awake = true;
  double calm = 0;  // s seguidos despierta, quieta y lejos de los robots
};

// Un subpaso de una baldosa; devuelve true si cambia de estado.
// Despierta: gap es su distancia al robot más cercano y max_speed la |v| máxima de sus nudos; se duerme tras
// settle_time s seguidos con gap > sleep_radius y max_speed < sleep_speed.
// Dormida: se despierta con gap < wake_radius o si max_shift (lo más que se ha movido desde que se durmió el nudo
// de fuera de uno de sus tramos frontera) supera wake_shift: la parte despierta tira de ella o la empuja. No vale la
// longitud del tramo: es casi inextensible y, con un extremo fijo, gira en vez de estirarse.
bool step_tile(TileState& s, const NetConfig::Sleep& cfg, double dt, double gap, double max_speed, double max_shift);

}  // namespace stonefish_net::core
