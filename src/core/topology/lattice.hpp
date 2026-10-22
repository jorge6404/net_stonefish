// Retícula de rombo: nudo (i, j) con i + j par; tramos de hilo (i, j)–(i+1, j±1).
#pragma once

#include <vector>

#include "stonefish_net/core/net_config.hpp"
#include "stonefish_net/core/net_topology.hpp"

namespace stonefish_net::core::topology
{

struct Lattice
{
  int nI = 0, nJ = 0;       // índices máximos (siempre pares: las esquinas son nudos)
  double hy = 0, hz = 0;    // semidiagonales del rombo
  std::vector<int> grid;    // (nI+1)·(nJ+1) índices de nudo, −1 donde no hay nudo

  int at(int i, int j) const;
  int corner(bool right, bool bottom) const { return at(right ? nI : 0, bottom ? nJ : 0); }
};

// Nudos, hilos de la red y sus diagonales continuas (para el render).
Lattice build_lattice(const NetConfig& c, NetTopology& t);

// Cabo de marco cerrado por los nudos del borde.
void add_frame(const NetConfig& c, const Lattice& lat, NetTopology& t);

// Nudos del borde j (i par) más cercanos a cada múltiplo de spacing desde el extremo izquierdo.
std::vector<int> along_edge(const Lattice& lat, int j, double spacing);

}  // namespace stonefish_net::core::topology
