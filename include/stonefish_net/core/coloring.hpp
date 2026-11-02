// Reparto de los tramos de la red en colores para resolverlos en paralelo: dos tramos del mismo color no
// comparten nudo, así que la GPU corrige todos los de un color a la vez sin que dos hilos toquen el mismo nudo.
#pragma once

#include <vector>

#include "stonefish_net/core/net_topology.hpp"

namespace stonefish_net::core
{

// Índice j de order: j < links.size() es topo.links[j]; si no, la diagonal topo.shear[j − links.size()].
struct LinkColoring
{
  std::vector<int> order;        // índices de tramo agrupados por color (dentro de un color, de menor a mayor)
  std::vector<int> color_begin;  // colores + 1: los tramos del color c son order[color_begin[c], color_begin[c + 1])
  int contact_colors = 0;        // los primeros colores, los de topo.links; detrás, los de las diagonales
};

// Voraz: los tramos de nudos con más tramos primero; cada uno toma el primer color libre en sus dos nudos. Las
// diagonales (topo.shear) se colorean aparte, cadena a cadena, en colores detrás de los de los tramos, para que el
// contacto (que solo mira los tramos) recorra los suyos sin pasar por ellas.
// Lanza std::runtime_error si harían falta más de 64 colores (un nudo con más de ~32 tramos).
LinkColoring color_links(const NetTopology& topo);

}  // namespace stonefish_net::core
