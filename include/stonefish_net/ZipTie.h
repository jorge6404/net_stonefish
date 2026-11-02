// Articulated zip tie, initial right grasp and optional software attachment.
#pragma once

#include <array>
#include <cstddef>
#include <map>
#include <memory>
#include <string>
#include <utility>

#include <Stonefish/actuators/Servo.h>

namespace sf
{
class SimulationManager;
}

namespace stonefish_net
{
class ZipTieGrasp;

class ZipTie
{
public:
  using Commands = std::map<std::string, std::pair<sf::ServoControlMode, sf::Scalar>>;

  // No <zip_tie> in the selected scene means no extra bodies or changes to the robot.
  static std::unique_ptr<ZipTie> Create(sf::SimulationManager& sm, const std::string& scenario);
  ~ZipTie();
  const std::string& robot_name() const { return robot_; }
  std::size_t segment_count() const { return segment_count_; }
  void InitializeCommands(Commands& commands) const;
  void HandleCommands(const Commands& commands);
  void UpdateGrasp(const Commands& commands);

private:
  ZipTie();
  struct Finger
  {
    sf::Servo* servo = nullptr;  // Owned by Stonefish, as are all bodies and joints.
    sf::Scalar target = 0;
    sf::Scalar original_torque = 0;
    bool initial_grasp = true;
  };
  std::string robot_;
  std::size_t segment_count_ = 0;
  std::array<Finger, 2> fingers_;
  std::unique_ptr<ZipTieGrasp> grasp_;
};

}  // namespace stonefish_net
