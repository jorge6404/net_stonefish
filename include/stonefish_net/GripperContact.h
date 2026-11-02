#pragma once

#include <unordered_set>
#include <map>
#include <string>
#include <vector>
#include <Stonefish/actuators/Servo.h>
#include "stonefish_net/core/grip_hold.hpp"

class btCollisionObject;
class btManifoldPoint;
namespace sf
{
class SimulationManager;
class SolidEntity;
}

namespace stonefish_net
{
// Native contact settings for every object touched by the configured fingers.
class GripperContact : public sf::Actuator
{
public:
  static GripperContact* Create(sf::SimulationManager& simulation, const std::string& scenario);
  bool contains(const sf::SolidEntity* link) const { return links_.count(link) != 0; }
  double static_friction() const { return static_; }
  double dynamic_friction() const { return dynamic_; }
  void ConfigureContact(btManifoldPoint& point, const btCollisionObject* a, const btCollisionObject* b) const;
  void Update(sf::Scalar dt) override;
  using Commands = std::map<std::string, std::pair<sf::ServoControlMode, sf::Scalar>>;
  void HandleCommands(const Commands& commands);
  sf::ActuatorType getType() const override { return sf::ActuatorType::PUSH; }
  std::vector<sf::Renderable> Render() override;

private:
  explicit GripperContact(sf::SimulationManager& simulation);
  sf::SimulationManager& simulation_;
  std::unordered_set<const sf::SolidEntity*> links_;
  std::unordered_set<const btCollisionObject*> colliders_;
  struct Finger
  {
    sf::SolidEntity* link;
    sf::Servo* servo;
    double close_sign;
    core::GripHold hold;
  };
  std::vector<Finger> fingers_;
  bool loaded(const Finger& finger) const;
  double hold_preload_ = 0.015;
  double static_ = 1.0, dynamic_ = 0.75, patch_radius_ = 0.004;
};
}  // namespace stonefish_net
