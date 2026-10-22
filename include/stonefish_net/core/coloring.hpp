// Reparto de los tramos de la red en colores para resolverlos en paralelo: dos tramos del mismo color no
// comparten nudo, así que la GPU corrige todos los de un color a la vez sin que dos hilos toquen el mismo nudo.
#pragma once

#include <vector>

#include "stonefish_net/core/net_topology.hpp"

namespace stonefish_net::core
{

struct LinkColoring
{
  std::vector<int> order;        // índices de tramo agrupados por color (dentro de un color, de menor a mayor)
  std::vector<int> color_begin;  // colores + 1: los tramos del color c son order[color_begin[c], color_begin[c + 1])
};

// Voraz: los tramos de nudos con más tramos primero; cada uno toma el primer color libre en sus dos nudos.
// Lanza std::runtime_error si harían falta más de 64 colores (un nudo con más de ~32 tramos).
LinkColoring color_links(const NetTopology& topo);

}  // namespace stonefish_net::core
