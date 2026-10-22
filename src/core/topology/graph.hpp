// Operaciones de grafo sobre NetTopology, comunes a todas las etapas de construcción.
#pragma once

#include <algorithm>
#include <utility>

#include "stonefish_net/core/net_topology.hpp"

namespace stonefish_net::core::topology
{

using Edge = std::pair<int, int>;  // nudos ordenados (menor, mayor)
inline Edge edge(int a, int b) { return {std::min(a, b), std::max(a, b)}; }

int add_node(NetTopology& t, const Eigen::Vector3d& position, NodeKind kind);
// Longitud natural = distancia actual entre a y b (ver Link::rest_length).
void add_link(NetTopology& t, int a, int b, double diameter, LinkKind kind);

// Se queda con la pieza conectada a `root`: descarta el resto, renumera nudos, tramos e hilos dibujados
// (partiéndolos donde falta un tramo) y los puntos de referencia, todo en una sola operación.
void prune_to_component(NetTopology& t, int root);

}  // namespace stonefish_net::core::topology
