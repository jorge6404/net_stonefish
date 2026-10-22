// Aparejo de la red: orinques hasta las boyas y líneas de fondeo hasta los bloques de anclaje.
#pragma once

#include "lattice.hpp"

namespace stonefish_net::core::topology
{

// Un orinque por boya desde la relinga superior; rellena t.marks.buoys.
void add_droppers(const NetConfig& c, const Lattice& lat, NetTopology& t);

// Una línea por esquina (inferiores con c.mooring.bottom, superiores con c.mooring.top); rellena t.marks.moorings
// en el orden de Landmarks::moorings.
// Lanza std::invalid_argument("mooring.floor_z: …") si un ancla no queda por debajo de su esquina.
void add_moorings(const NetConfig& c, const Lattice& lat, NetTopology& t);

}  // namespace stonefish_net::core::topology
