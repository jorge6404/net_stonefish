// Daño aleatorio reproducible: rotos que crecen sobre rombos vecinos sin tocar el marco (con cabos sueltos
// en su borde) y una fracción de hilos cortados al azar.
#pragma once

#include "lattice.hpp"

namespace stonefish_net::core::topology
{

// Quita los hilos rotos y cortados y anota en t.damage el tamaño de cada roto y los hilos sueltos cortados.
// Puede dejar piezas sueltas: las recorta prune_to_component.
void add_damage(const NetConfig& c, const Lattice& lat, NetTopology& t);

}  // namespace stonefish_net::core::topology
