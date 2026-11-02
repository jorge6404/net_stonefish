#pragma once

#include <array>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <Stonefish/actuators/Servo.h>
#include "stonefish_net/core/grasp_latch.hpp"

class btMultiBodyFixedConstraint;
namespace tinyxml2 { class XMLElement; }
namespace sf
{
class SimulationManager;
class FeatherstoneRobot;
class FeatherstoneEntity;
class SolidEntity;
}

namespace stonefish_net
{
// Optional, zip-tie-only attachment. Normal finger commands remain untouched.
class ZipTieGrasp
{
public:
  using Commands = std::map<std::string, std::pair<sf::ServoControlMode, sf::Scalar>>;
  static std::unique_ptr<ZipTieGrasp> Create(sf::SimulationManager& sm, sf::FeatherstoneRobot& robot,
    const tinyxml2::XMLElement& zip_config, const std::vector<sf::SolidEntity*>& pieces,
    sf::SolidEntity& initial_piece);
  ~ZipTieGrasp();
  void Update(const Commands& commands);
  bool InitiallyAttached() const { return initially_attached_; }

private:
  ZipTieGrasp(sf::SimulationManager& sm, sf::FeatherstoneEntity& dynamics);
  struct Gripper
  {
    std::string name;
    sf::SolidEntity* palm = nullptr;
    int palm_index = -2;  // Bullet index: Stonefish link index minus one.
    std::array<sf::SolidEntity*, 2> fingers{};
    std::array<sf::Servo*, 2> servos{};
    std::array<double, 2> open{}, closed{};
    std::array<core::GraspPositionIntent, 2> position_intent;
    core::GraspLatch latch;
  };
  struct CollisionPair
  {
    sf::SolidEntity* piece;
    sf::SolidEntity* link;
    std::size_t owner;
  };
  sf::SolidEntity* Candidate(const Gripper& gripper) const;
  bool Attach(std::size_t owner, sf::SolidEntity& piece);
  void Release();
  void RestoreCollisions(bool force);

  sf::SimulationManager& sm_;
  sf::FeatherstoneEntity& dynamics_;
  std::vector<sf::SolidEntity*> pieces_;
  std::vector<Gripper> grippers_;
  std::vector<CollisionPair> disabled_;
  std::unique_ptr<btMultiBodyFixedConstraint> constraint_;
  std::size_t owner_ = 0;
  double max_distance_ = 0.01, min_closure_ = 0.2, release_angle_ = 0.03;
  double max_force_ = 100, max_torque_ = 2, erp_ = 0.1;
  bool initially_attached_ = false;
  sf::Vector3 center_, half_extents_;
};
}  // namespace stonefish_net
