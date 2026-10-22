#include "stonefish_net/core/buoy_asset.hpp"

#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace stonefish_net::core
{

BuoyAsset BuoyAsset::load(const std::string& dir)
{
  const std::string meta_path = dir + "/buoy.yaml";
  try {
    const YAML::Node meta = YAML::LoadFile(meta_path);
    const YAML::Node eye = meta["eye"];
    if (!eye || !eye.IsSequence() || eye.size() != 3) {
      throw std::invalid_argument(meta_path + ": eye debe ser [x, y, z]");
    }
    return BuoyAsset{dir + "/buoy_vis.obj", dir + "/buoy_phy.obj",
                     Eigen::Vector3d(eye[0].as<double>(), eye[1].as<double>(), eye[2].as<double>())};
  } catch (const YAML::Exception& e) {
    throw std::invalid_argument(meta_path + ": " + e.what());
  }
}

}  // namespace stonefish_net::core
