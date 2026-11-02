#include "stonefish_net/ZipTieGrasp.h"
#include "stonefish_net/GraspConstraint.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <Stonefish/core/FeatherstoneRobot.h>
#include <Stonefish/core/SimulationManager.h>
#include <Stonefish/entities/FeatherstoneEntity.h>
#include <Stonefish/entities/SolidEntity.h>
#include <Stonefish/tinyxml2.h>
#include <BulletSoftBody/btSoftMultiBodyDynamicsWorld.h>
#include <BulletDynamics/Featherstone/btMultiBodyFixedConstraint.h>

namespace stonefish_net
{
namespace
{
[[noreturn]] void invalid(const std::string& message)
{
  throw std::invalid_argument("zip_tie grasp_assist: " + message);
}

std::string attribute(const tinyxml2::XMLElement& element, const char* key)
{
  const char* value = element.Attribute(key);
  if (!value || !*value) invalid(std::string("missing ") + key);
  return value;
}

double number(const tinyxml2::XMLElement& element, const char* key, double fallback)
{
  if (!element.Attribute(key)) return fallback;
  double value;
  if (element.QueryDoubleAttribute(key, &value) != tinyxml2::XML_SUCCESS || !std::isfinite(value)) {
    invalid(std::string("invalid ") + key);
  }
  return value;
}

sf::Vector3 vector(const tinyxml2::XMLElement& element, const char* key)
{
  std::istringstream input(attribute(element, key));
  double x, y, z;
  std::string excess;
  if (!(input >> x >> y >> z) || (input >> excess) ||
      !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) invalid(std::string("invalid ") + key);
  return sf::Vector3(x, y, z);
}

btCollisionObject* collider(sf::SolidEntity& solid)
{
  if (solid.getRigidBody()) return solid.getRigidBody();
  return solid.getMultiBodyLinkCollider();
}

// A geometric proximity query, independent of contact impulses and friction.
struct Proximity : btCollisionWorld::ContactResultCallback
{
  Proximity(sf::SolidEntity& piece, double threshold) : piece_(collider(piece))
  {
    m_closestDistanceThreshold = threshold;
  }
  btScalar addSingleResult(btManifoldPoint& contact, const btCollisionObjectWrapper* a, int, int,
                           const btCollisionObjectWrapper*, int, int) override
  {
    if (contact.getDistance() < minimum) {
      minimum = contact.getDistance();
      point = a->getCollisionObject() == piece_ ? contact.getPositionWorldOnA() : contact.getPositionWorldOnB();
    }
    return 0;
  }
  const btCollisionObject* piece_;
  double minimum = BT_LARGE_FLOAT;
  sf::Vector3 point = sf::V0();
};

Proximity proximity(sf::SimulationManager& sm, sf::SolidEntity& piece, sf::SolidEntity& finger, double threshold)
{
  Proximity result(piece, threshold);
  sm.getDynamicsWorld()->contactPairTest(collider(piece), collider(finger), result);
  return result;
}
}  // namespace

ZipTieGrasp::ZipTieGrasp(sf::SimulationManager& sm, sf::FeatherstoneEntity& dynamics)
  : sm_(sm), dynamics_(dynamics), center_(sf::V0()), half_extents_(sf::V0()) {}

std::unique_ptr<ZipTieGrasp> ZipTieGrasp::Create(sf::SimulationManager& sm, sf::FeatherstoneRobot& robot,
  const tinyxml2::XMLElement& zip_config, const std::vector<sf::SolidEntity*>& pieces,
  sf::SolidEntity& initial_piece)
{
  const auto* config = zip_config.FirstChildElement("grasp_assist");
  if (!config) return nullptr;
  bool enabled = false;
  if (config->QueryBoolAttribute("enabled", &enabled) != tinyxml2::XML_SUCCESS) invalid("enabled must be a boolean");
  if (!enabled) return nullptr;
  auto result = std::unique_ptr<ZipTieGrasp>(new ZipTieGrasp(sm, *robot.getDynamics()));
  result->pieces_ = pieces;
  result->max_distance_ = number(*config, "max_distance", 0.01);
  result->min_closure_ = number(*config, "min_closure", 0.2);
  result->release_angle_ = number(*config, "release_angle", 0.03);
  result->max_force_ = number(*config, "max_force", 100);
  result->max_torque_ = number(*config, "max_torque", 2);
  result->erp_ = number(*config, "erp", 0.1);
  result->center_ = vector(*config, "capture_xyz");
  result->half_extents_ = vector(*config, "capture_half_extents");
  if (result->max_distance_ <= 0 || result->max_distance_ > 0.03 ||
      result->min_closure_ <= 0 || result->min_closure_ >= 1 ||
      result->release_angle_ <= 0 || result->release_angle_ > 0.2 ||
      result->max_force_ <= 0 || result->max_force_ > 10000 ||
      result->max_torque_ <= 0 || result->max_torque_ > 100 ||
      result->erp_ <= 0 || result->erp_ > 0.5 ||
      result->half_extents_[result->half_extents_.minAxis()] <= 0) invalid("parameters outside valid range");

  for (const auto* entry = config->FirstChildElement("gripper"); entry; entry = entry->NextSiblingElement("gripper")) {
    Gripper gripper;
    gripper.name = attribute(*entry, "palm");
    const std::string prefix = robot.getName() + "/";
    gripper.palm = robot.getLink(prefix + gripper.name);
    gripper.palm_index = robot.getLinkIndex(prefix + gripper.name);
    if (!gripper.palm || !collider(*gripper.palm) || gripper.palm_index < -1) invalid("palm not found");
    for (std::size_t i = 0; i < 2; ++i) {
      const std::string suffix = i == 0 ? "a" : "b";
      gripper.fingers[i] = robot.getLink(prefix + attribute(*entry, ("finger_" + suffix).c_str()));
      gripper.servos[i] = dynamic_cast<sf::Servo*>(robot.getActuator(prefix + attribute(*entry, ("servo_" + suffix).c_str())));
      if (!gripper.fingers[i] || !collider(*gripper.fingers[i]) || !gripper.servos[i]) invalid("finger or servo not found");
      gripper.open[i] = number(*entry, ("open_" + suffix).c_str(), i == 0 ? -0.5 : 0.5);
      gripper.closed[i] = number(*entry, ("closed_" + suffix).c_str(), i == 0 ? 0.125 : -0.125);
      if (gripper.open[i] == gripper.closed[i]) invalid("open and closed angles must differ");
    }
    for (const auto& previous : result->grippers_) {
      if (previous.palm == gripper.palm) invalid("duplicate gripper");
    }
    result->grippers_.push_back(gripper);
  }
  if (result->grippers_.empty()) invalid("no grippers configured");
  bool initial_grasp = false;
  if (config->Attribute("initial_grasp") &&
      config->QueryBoolAttribute("initial_grasp", &initial_grasp) != tinyxml2::XML_SUCCESS) {
    invalid("initial_grasp must be a boolean");
  }
  if (initial_grasp && !pieces.empty()) {
    const auto initial_palm = attribute(zip_config, "palm");
    for (std::size_t index = 0; index < result->grippers_.size(); ++index) {
      auto& gripper = result->grippers_[index];
      if (gripper.name != initial_palm) continue;
      result->initially_attached_ = result->Attach(index, initial_piece);
      if (result->initially_attached_) gripper.latch.update(true, false, true, true);
      break;
    }
  }
  return result;
}

ZipTieGrasp::~ZipTieGrasp()
{
  Release();
  RestoreCollisions(true);
}

sf::SolidEntity* ZipTieGrasp::Candidate(const Gripper& gripper) const
{
  sf::SolidEntity* best = nullptr;
  double best_distance = BT_LARGE_FLOAT;
  const sf::Transform palm_inverse = gripper.palm->getOTransform().inverse();
  for (auto* piece : pieces_) {
    const auto a = proximity(sm_, *piece, *gripper.fingers[0], max_distance_);
    if (a.minimum > max_distance_) continue;
    const auto b = proximity(sm_, *piece, *gripper.fingers[1], max_distance_);
    if (b.minimum > max_distance_) continue;
    const sf::Vector3 local = palm_inverse * ((a.point + b.point) * 0.5) - center_;
    if (btFabs(local.x()) > half_extents_.x() || btFabs(local.y()) > half_extents_.y() ||
        btFabs(local.z()) > half_extents_.z()) continue;
    const double score = std::max(a.minimum, b.minimum);
    if (score < best_distance) {
      best_distance = score;
      best = piece;
    }
  }
  return best;
}

bool ZipTieGrasp::Attach(std::size_t owner, sf::SolidEntity& piece)
{
  auto& gripper = grippers_[owner];
  // post_tick precedes synchronization of graphical motion states. Construct
  // from Bullet's CURRENT physical transforms, never SolidEntity::getCGTransform.
  const auto& body = piece.getRigidBody()->getCenterOfMassTransform();
  for (int axis = 0; axis < 3; ++axis) {
    if (!std::isfinite(body.getOrigin()[axis])) return false;
    for (int column = 0; column < 3; ++column) {
      if (!std::isfinite(body.getBasis()[axis][column])) return false;
    }
  }
  auto constraint = std::make_unique<GraspConstraint>(*dynamics_.getMultiBody(), gripper.palm_index,
    *piece.getRigidBody(), max_force_, max_torque_, erp_);
  const sf::Vector3 pivot = dynamics_.getMultiBody()->localPosToWorld(gripper.palm_index, constraint->getPivotInA());
  const double pivot_error = (pivot - body.getOrigin()).length2();
  const sf::Matrix3 frame = dynamics_.getMultiBody()->localFrameToWorld(gripper.palm_index, constraint->getFrameInA());
  double frame_error = 0;
  for (int axis = 0; axis < 3; ++axis) {
    frame_error += (frame.getColumn(axis) - body.getBasis().getColumn(axis)).length2();
  }
  if (!std::isfinite(pivot_error) || !std::isfinite(frame_error) || pivot_error > 1e-12 || frame_error > 1e-12) return false;
  constraint_ = std::move(constraint);
  owner_ = owner;

  // Ignore only zip-tie/holding-gripper pairs. Otherwise the fingers' ordinary
  // close command fights the fixed grasp. Preserve pre-existing exclusions.
  for (auto* body_piece : pieces_) {
    for (auto* link : {gripper.palm, gripper.fingers[0], gripper.fingers[1]}) {
      const bool listed = sm_.CheckCollision(body_piece, link) >= 0;
      const bool enabled = sm_.getCollisionFilter() == sf::CollisionFilter::EXCLUSIVE ? !listed : listed;
      if (!enabled) continue;
      sm_.DisableCollision(body_piece, link);
      disabled_.push_back({body_piece, link, owner});
      auto* dispatcher = sm_.getDynamicsWorld()->getDispatcher();
      for (int m = 0; m < dispatcher->getNumManifolds(); ++m) {
        auto* manifold = dispatcher->getManifoldByIndexInternal(m);
        if ((manifold->getBody0() == collider(*body_piece) && manifold->getBody1() == collider(*link)) ||
            (manifold->getBody1() == collider(*body_piece) && manifold->getBody0() == collider(*link))) {
          dispatcher->clearManifold(manifold);
        }
      }
    }
  }
  sm_.getDynamicsWorld()->addMultiBodyConstraint(constraint_.get());
  piece.getRigidBody()->activate(true);
  dynamics_.getMultiBody()->wakeUp();
  std::clog << "[zip_tie] Agarre por software: " << gripper.name << " -> " << piece.getName() << std::endl;
  return true;
}

void ZipTieGrasp::Release()
{
  if (!constraint_) return;
  sm_.getDynamicsWorld()->removeMultiBodyConstraint(constraint_.get());
  constraint_.reset();
  std::clog << "[zip_tie] Brida liberada de " << grippers_[owner_].name << std::endl;
}

void ZipTieGrasp::RestoreCollisions(bool force)
{
  for (auto it = disabled_.begin(); it != disabled_.end();) {
    if (!force && ((constraint_ && owner_ == it->owner) ||
        proximity(sm_, *it->piece, *it->link, 0.0002).minimum < 0.0002)) {
      ++it;
      continue;
    }
    sm_.EnableCollision(it->piece, it->link);
    it = disabled_.erase(it);
  }
}

void ZipTieGrasp::Update(const Commands& commands)
{
  RestoreCollisions(false);
  for (std::size_t index = 0; index < grippers_.size(); ++index) {
    auto& gripper = grippers_[index];
    bool closing = false, opening = false, closed = true;
    for (std::size_t i = 0; i < 2; ++i) {
      const double angle = gripper.servos[i]->getPosition();
      const double sign = gripper.closed[i] > gripper.open[i] ? 1 : -1;
      closed = closed && std::isfinite(angle) &&
        (angle - gripper.open[i]) / (gripper.closed[i] - gripper.open[i]) >= min_closure_;
      const auto command = commands.find(gripper.servos[i]->getJointName());
      if (command == commands.end() || !std::isfinite(command->second.second)) continue;
      if (command->second.first == sf::ServoControlMode::VELOCITY) {
        gripper.position_intent[i].reset();
        closing = closing || command->second.second * sign > 1e-6;
        opening = opening || command->second.second * sign < -1e-6;
      } else if (command->second.first == sf::ServoControlMode::POSITION && std::isfinite(angle)) {
        const auto intent = gripper.position_intent[i].update(command->second.second, angle, sign, release_angle_);
        closing = closing || intent == core::GraspPositionIntent::Direction::Closing;
        opening = opening || intent == core::GraspPositionIntent::Direction::Opening;
      }
    }
    sf::SolidEntity* candidate = !constraint_ && closed && !opening && (closing || gripper.latch.armed())
      ? Candidate(gripper) : nullptr;
    const auto action = gripper.latch.update(closing, opening, candidate != nullptr, closed);
    if (action == core::GraspLatch::Action::Release && index == owner_) Release();
    else if (action == core::GraspLatch::Action::Attach && !Attach(index, *candidate)) {
      gripper.latch.update(false, true, false, false);
    }
  }
}
}  // namespace stonefish_net
