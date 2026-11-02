#pragma once

#include <cmath>

namespace stonefish_net::core
{

// A stopped closing velocity should keep pressure only if contact was loaded.
// Opening, position commands and repeated zero commands never add preload.
class GripHold
{
public:
  bool update(bool velocity_mode, double command, double close_sign, bool loaded)
  {
    const bool preload = closing_ && velocity_mode && std::abs(command) < 1e-9 && loaded;
    closing_ = velocity_mode && command * close_sign > 1e-9;
    return preload;
  }
private:
  bool closing_ = false;
};

}  // namespace stonefish_net::core
