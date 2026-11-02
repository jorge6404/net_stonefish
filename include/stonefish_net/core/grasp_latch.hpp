// Command-driven grasp latch, independent of ROS, geometry and Bullet.
#pragma once

#include <optional>

namespace stonefish_net::core
{

// Interpret position commands rather than servo tracking error. Repeating a
// fixed target must not count as another closing command after an opening.
class GraspPositionIntent
{
public:
  enum class Direction { None, Closing, Opening };

  // The caller validates finite inputs, close_sign (+1 or -1), and a positive
  // release_angle. reset() starts a new sequence after velocity control.
  Direction update(double target, double actual, double close_sign, double release_angle)
  {
    if (!reference_) reference_ = actual * close_sign;
    const double signed_target = target * close_sign;
    const double delta = signed_target - *reference_;
    if (delta > 1e-6) {
      reference_ = signed_target;
      return Direction::Closing;
    }
    if (delta < -release_angle) {
      reference_ = signed_target;
      return Direction::Opening;
    }
    // Preserve the most closed reference through small opening increments.
    return Direction::None;
  }

  void reset() { reference_.reset(); }

private:
  std::optional<double> reference_;
};

class GraspLatch
{
public:
  enum class Action { None, Attach, Release };

  Action update(bool closing, bool opening, bool candidate_inside, bool sufficiently_closed)
  {
    // Opening always wins, including contradictory commands from the two fingers.
    // Disarm so that a following zero command cannot immediately reattach.
    if (opening) {
      armed_ = false;
      if (attached_) {
        attached_ = false;
        return Action::Release;
      }
      return Action::None;
    }
    if (closing) armed_ = true;
    if (!attached_ && armed_ && candidate_inside && sufficiently_closed) {
      attached_ = true;
      return Action::Attach;
    }
    // Once attached, proximity and contact are no longer required. Only opening
    // releases the object; zero commands also preserve a pending closing attempt.
    return Action::None;
  }

  bool attached() const { return attached_; }
  bool armed() const { return armed_; }

private:
  bool armed_ = false;
  bool attached_ = false;
};

}  // namespace stonefish_net::core
