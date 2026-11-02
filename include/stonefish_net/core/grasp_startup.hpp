// Contact-based completion of the initial grasp, independent of ROS and Bullet.
#pragma once

#include <array>

namespace stonefish_net::core
{

class GraspStartup
{
public:
  enum class State { Closing, Ready, Failed };

  State update(double dt, const std::array<double, 2>& normal_forces, bool steady)
  {
    if (state_ != State::Closing) return state_;
    elapsed_ += dt;
    if (steady && normal_forces[0] >= kMinimumForce && normal_forces[1] >= kMinimumForce) {
      loaded_time_ += dt;
    } else {
      loaded_time_ = 0;
    }
    if (loaded_time_ + 1e-12 >= kLoadedTime) state_ = State::Ready;
    else if (elapsed_ + 1e-12 >= kTimeout) state_ = State::Failed;
    return state_;
  }

  State state() const { return state_; }

private:
  static constexpr double kMinimumForce = 0.02;  // N per finger; a proximity alone is not a grasp.
  static constexpr double kLoadedTime = 0.02;   // s of uninterrupted loaded, settled contact.
  static constexpr double kTimeout = 2.0;       // simulated seconds, before normal dynamics.
  State state_ = State::Closing;
  double elapsed_ = 0;
  double loaded_time_ = 0;
};

}  // namespace stonefish_net::core
