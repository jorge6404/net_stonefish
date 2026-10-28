// La red empaquetada en arrays planos de float/int con el formato std430 de los shaders: lo que se sube a la
// GPU y lo que usa el subpaso de referencia (xpbd_ref) para hacer exactamente las mismas cuentas.
// Los tramos van en el orden de los colores (LinkColoring::order): el tramo i del layout es order[i].
#pragma once

#include <vector>

#include "stonefish_net/core/coloring.hpp"
#include "stonefish_net/core/net_topology.hpp"

namespace stonefish_net::core
{

struct GpuLayout
{
  std::vector<float> x0;          // 4 por nudo: posición inicial, w = 1 / masa inercial (0 si es fijo)
  std::vector<float> node_props;  // 4 por nudo: masa, volumen, masa inercial, Ø del nudo
  std::vector<int> links;         // 2 por tramo: nudos a, b
  std::vector<float> link_props;  // 4 por tramo: longitud natural, diámetro, compliance (m/N), 0
  std::vector<int> adj_begin;     // nudos + 1: tramos del nudo k en adj_links[adj_begin[k], adj_begin[k + 1])
  std::vector<int> adj_links;     // índices de tramo del layout
  std::vector<int> color_begin;   // = LinkColoring::color_begin

  size_t node_count() const { return x0.size() / 4; }
  size_t link_count() const { return links.size() / 2; }
  int colors() const { return static_cast<int>(color_begin.size()) - 1; }
};

GpuLayout make_gpu_layout(const NetTopology& topo, const LinkColoring& coloring);

}  // namespace stonefish_net::core
