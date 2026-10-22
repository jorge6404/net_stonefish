#include "stonefish_net/core/current_grid.hpp"

#include <algorithm>
#include <cmath>

namespace stonefish_net::core
{

CurrentGrid current_grid_for(const std::vector<Eigen::Vector3f>& x, float cell)
{
  // Solo los nudos finitos: uno con NaN no puede decidir el tamaño
  bool any = false;
  Eigen::Vector3f lo = Eigen::Vector3f::Zero(), hi = lo;
  for (const auto& p : x) {
    if (!p.allFinite()) continue;
    lo = any ? lo.cwiseMin(p) : p;
    hi = any ? hi.cwiseMax(p) : p;
    any = true;
  }
  if (!any) {
    return {Eigen::Vector3f::Zero(), cell, {1, 1, 1}};
  }
  // Si un nudo se ha ido lejos, celdas más grandes antes que una rejilla enorme (cada punto es una consulta al
  // océano en cada tick)
  const float extent = (hi - lo).maxCoeff();
  const float size = std::max(cell, extent / static_cast<float>(kMaxCurrentGrid - 3));
  CurrentGrid g{lo - Eigen::Vector3f::Constant(size), size, {1, 1, 1}};
  for (int a = 0; a < 3; ++a) {
    g.dims[a] = std::min(static_cast<int>(std::ceil((hi[a] - lo[a]) / size)) + 3, kMaxCurrentGrid);
  }
  return g;
}

}  // namespace stonefish_net::core
