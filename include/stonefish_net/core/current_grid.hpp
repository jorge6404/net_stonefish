// Rejilla en la que el solver de GPU muestrea la corriente del océano: cubre las posiciones de la red con una celda
// de margen por cada lado.
#pragma once

#include <vector>

#include <Eigen/Core>

namespace stonefish_net::core
{

inline constexpr int kMaxCurrentGrid = 128;  // puntos por eje como mucho

struct CurrentGrid
{
  Eigen::Vector3f origin;  // primer punto, m
  float cell;              // separación, m
  int dims[3];             // puntos por eje
};

CurrentGrid current_grid_for(const std::vector<Eigen::Vector3f>& x, float cell);

}  // namespace stonefish_net::core
