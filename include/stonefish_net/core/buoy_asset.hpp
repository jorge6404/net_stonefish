// Asset de la boya generado por tools/buoy_asset.py (data/buoy/): mallas y posición del ojal.
// La textura (buoy.png) la usa el look net_buoy del escenario.
#pragma once

#include <string>

#include <Eigen/Core>

namespace stonefish_net::core
{

struct BuoyAsset
{
  std::string visual_mesh, physics_mesh;  // rutas absolutas
  Eigen::Vector3d eye;                    // ojal en el marco de la malla (NED, ojal arriba)

  // Lanza std::invalid_argument("<dir>/buoy.yaml: …") si falta el fichero o el campo eye.
  static BuoyAsset load(const std::string& dir);
};

}  // namespace stonefish_net::core
