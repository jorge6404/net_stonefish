// Red irregular (sección irregular del YAML): campos aleatorios suaves sobre el plano de la red, para que el ángulo
// de reposo de los rombos y la longitud natural de los hilos varíen por zonas, como en una red real.
#pragma once

#include <Eigen/Core>

#include "stonefish_net/core/net_config.hpp"
#include "stonefish_net/core/net_topology.hpp"

namespace stonefish_net::core::topology
{

// Campo gaussiano de media 0 y varianza 1 en (y, z), con correlación exp(−r² / (2 scale²)): suma de kModes cosenos
// de número de onda aleatorio. Reproducible con la semilla.
class RandomField
{
public:
  RandomField(double scale, unsigned seed);
  double operator()(double y, double z) const;

private:
  static constexpr int kModes = 64;
  Eigen::Matrix<double, kModes, 2> k_;
  Eigen::Matrix<double, kModes, 1> phase_;
};

// Longitud natural de cada hilo × (1 + irregular.length · campo en su punto medio). Los cabos no cambian.
void vary_twine_length(const NetConfig& c, NetTopology& t);

// El campo de los ángulos de reposo: otra semilla que el de las longitudes
RandomField angle_field(const NetConfig& c);

// Ángulo de reposo (rad) del rombo de centro (y, z), en las esquinas de su diagonal más vertical: el que tiene más
// irregular.angle_deg · campo, entre 10° y 170°
double rest_angle(const NetConfig& c, const RandomField& field, double angle_rad, double y, double z);

}  // namespace stonefish_net::core::topology
