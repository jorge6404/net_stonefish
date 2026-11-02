#include "stonefish_net/GripperContact.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <Stonefish/actuators/Servo.h>
#include <Stonefish/core/Robot.h>
#include <Stonefish/core/SimulationManager.h>
#include <Stonefish/entities/SolidEntity.h>
#include <Stonefish/tinyxml2.h>
#include <BulletDynamics/Featherstone/btMultiBodyLinkCollider.h>
#include <BulletCollision/CollisionShapes/btCompoundShape.h>
#include <BulletCollision/CollisionShapes/btConvexHullShape.h>

namespace stonefish_net
{
namespace
{
void optimize_finger(btCollisionShape& shape)
{
  if (shape.isCompound()) {
    auto& compound = static_cast<btCompoundShape&>(shape);
    for (int i = 0; i < compound.getNumChildShapes(); ++i) optimize_finger(*compound.getChildShape(i));
    compound.recalculateLocalAabb();
  } else if (shape.getShapeType() == CONVEX_HULL_SHAPE_PROXYTYPE) {
    // Keep the same convex surface with only its extreme vertices. Exact wire
    // queries should not scan all triangles/duplicate vertices of the OBJ.
    auto& hull = static_cast<btConvexHullShape&>(shape);
    hull.optimizeConvexHull();
    hull.recalcLocalAabb();
  }
}

std::string required(const tinyxml2::XMLElement& node, const char* key)
{
  const char* text = node.Attribute(key);
  if (!text || !*text) throw std::invalid_argument(std::string("gripper_contact: missing ") + key);
  return text;
}

double parameter(const tinyxml2::XMLElement& node, const char* key, double fallback)
{
  if (!node.Attribute(key)) return fallback;
  double value;
  if (node.QueryDoubleAttribute(key, &value) != tinyxml2::XML_SUCCESS || !std::isfinite(value) || value < 0) {
    throw std::invalid_argument(std::string("gripper_contact: invalid ") + key);
  }
  return value;
}

sf::Vector3 velocity(const btCollisionObject* object, const sf::Vector3& point)
{
  auto* entity = static_cast<sf::Entity*>(object->getUserPointer());
  if (entity && entity->getType() == sf::EntityType::SOLID) {
    const auto* solid = static_cast<sf::SolidEntity*>(entity);
    return solid->getLinearVelocityInLocalPoint(solid->getCGTransform().inverse() * point);
  }
  if (entity && entity->getType() == sf::EntityType::STATIC) return sf::V0();
  return object->getInterpolationLinearVelocity() +
    object->getInterpolationAngularVelocity().cross(point - object->getWorldTransform().getOrigin());
}
}  // namespace

GripperContact::GripperContact(sf::SimulationManager& simulation)
: sf::Actuator("gripper_contact"), simulation_(simulation) {}

std::vector<sf::Renderable> GripperContact::Render() { return {}; }

GripperContact* GripperContact::Create(sf::SimulationManager& simulation, const std::string& scenario)
{
  tinyxml2::XMLDocument document;
  if (document.LoadFile(scenario.c_str()) != tinyxml2::XML_SUCCESS) {
    throw std::invalid_argument("gripper_contact: cannot read scene");
  }
  const auto* root = document.FirstChildElement("scenario");
  const auto* config = root ? root->FirstChildElement("gripper_contact") : nullptr;
  if (!config) return nullptr;
  if (config->NextSiblingElement("gripper_contact")) throw std::invalid_argument("duplicate gripper_contact");
  std::unique_ptr<GripperContact> result(new GripperContact(simulation));
  result->static_ = parameter(*config, "static_friction", 1.0);
  result->dynamic_ = parameter(*config, "dynamic_friction", 0.75);
  result->patch_radius_ = parameter(*config, "patch_radius", 0.004);
  const double torque = parameter(*config, "max_torque", 0.5);
  result->hold_preload_ = parameter(*config, "hold_preload", 0.015);
  if (result->dynamic_ > result->static_ || torque <= 0 || result->patch_radius_ > 0.02 || result->hold_preload_ > 0.1) {
    throw std::invalid_argument("gripper_contact: invalid friction, patch radius or torque");
  }
  auto* robot = simulation.getRobot(required(*config, "robot"));
  if (!robot) throw std::invalid_argument("gripper_contact: robot not found");
  for (auto* finger = config->FirstChildElement("finger"); finger; finger = finger->NextSiblingElement("finger")) {
    auto* link = robot->getLink(robot->getName() + "/" + required(*finger, "link"));
    auto* servo = dynamic_cast<sf::Servo*>(robot->getActuator(robot->getName() + "/" + required(*finger, "servo")));
    if (!link || !servo) throw std::invalid_argument("gripper_contact: finger link/servo not found");
    const std::string closing = required(*finger, "close_sign");
    if (closing != "1" && closing != "-1") throw std::invalid_argument("gripper_contact: close_sign must be 1 or -1");
    btCollisionObject* collider = link->getRigidBody();
    if (!collider) collider = link->getMultiBodyLinkCollider();
    if (!collider) throw std::invalid_argument("gripper_contact: finger has no collision shape");
    if (!result->links_.insert(link).second) throw std::invalid_argument("gripper_contact: duplicate finger");
    result->colliders_.insert(collider);
    optimize_finger(*collider->getCollisionShape());
    result->fingers_.push_back({link, servo, closing == "1" ? 1.0 : -1.0, {}});
    collider->setCollisionFlags(collider->getCollisionFlags() | btCollisionObject::CF_HAS_FRICTION_ANCHOR);
    servo->setMaxTorque(torque);
  }
  if (result->links_.empty()) throw std::invalid_argument("gripper_contact: no fingers configured");
  return result.release();
}

bool GripperContact::loaded(const Finger& finger) const
{
  const btCollisionObject* collider = finger.link->getRigidBody();
  if (!collider) collider = finger.link->getMultiBodyLinkCollider();
  const double dt = simulation_.getDynamicsWorld()->getSolverInfo().m_timeStep;
  auto* dispatcher = simulation_.getDynamicsWorld()->getDispatcher();
  for (int i = 0; i < dispatcher->getNumManifolds(); ++i) {
    const auto* manifold = dispatcher->getManifoldByIndexInternal(i);
    if (manifold->getBody0() != collider && manifold->getBody1() != collider) continue;
    double impulse = 0;
    for (int j = 0; j < manifold->getNumContacts(); ++j) {
      const auto& point = manifold->getContactPoint(j);
      if (point.getDistance() < 0.0002) impulse += point.getAppliedImpulse();
    }
    if (impulse > 0.02 * dt) return true;
  }
  return false;
}

void GripperContact::HandleCommands(const Commands& commands)
{
  for (auto& finger : fingers_) {
    const auto command = commands.find(finger.servo->getJointName());
    if (command == commands.end()) continue;
    if (finger.hold.update(command->second.first == sf::ServoControlMode::VELOCITY,
                            command->second.second, finger.close_sign, loaded(finger))) {
      // ROS2SimulationManager has already applied the zero velocity command and
      // captured the current joint angle. Keep a small closing position offset,
      // bounded by the configured motor torque and the original joint limits.
      finger.servo->setDesiredPosition(finger.servo->getDesiredPosition() + finger.close_sign * hold_preload_);
    }
  }
}

void GripperContact::ConfigureContact(btManifoldPoint& point, const btCollisionObject* a,
                                      const btCollisionObject* b) const
{
  if ((!colliders_.count(a) && !colliders_.count(b)) || !a->getUserPointer() || !b->getUserPointer()) return;
  const sf::Vector3 relative = velocity(a, point.getPositionWorldOnA()) - velocity(b, point.getPositionWorldOnB());
  const sf::Vector3 tangent = relative - relative.dot(point.m_normalWorldOnB) * point.m_normalWorldOnB;
  const double transition = std::clamp((tangent.length() - 0.005) / 0.025, 0.0, 1.0);
  point.m_combinedFriction = static_ + transition * (dynamic_ - static_);
  point.m_contactPointFlags |= BT_CONTACT_FLAG_FRICTION_ANCHOR;
  // A finite pad patch can resist spin as well as translation. Torque remains
  // proportional to normal pressure and becomes zero when the fingers open.
  point.m_combinedSpinningFriction = point.m_combinedFriction * patch_radius_;
  point.m_combinedRollingFriction = point.m_combinedFriction * patch_radius_ * 0.5;
}

void GripperContact::Update(sf::Scalar dt)
{
  (void)dt;
  auto* dispatcher = simulation_.getDynamicsWorld()->getDispatcher();
  for (int i = 0; i < dispatcher->getNumManifolds(); ++i) {
    auto* manifold = dispatcher->getManifoldByIndexInternal(i);
    for (int j = 0; j < manifold->getNumContacts(); ++j) {
      ConfigureContact(manifold->getContactPoint(j), manifold->getBody0(), manifold->getBody1());
    }
  }
}
}  // namespace stonefish_net
