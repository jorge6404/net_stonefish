#include "stonefish_net/ZipTie.h"
#include "stonefish_net/ZipTieGrasp.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <yaml-cpp/yaml.h>
#include <Stonefish/tinyxml2.h>
#include <Stonefish/core/FeatherstoneRobot.h>
#include <Stonefish/core/MaterialManager.h>
#include <Stonefish/core/SimulationApp.h>
#include <Stonefish/core/SimulationManager.h>
#include <Stonefish/entities/FeatherstoneEntity.h>
#include <Stonefish/entities/solids/Compound.h>
#include <Stonefish/entities/solids/Polyhedron.h>
#include <Stonefish/joints/Joint.h>
#include <BulletSoftBody/btSoftMultiBodyDynamicsWorld.h>
#include <BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h>

#include "stonefish_net/core/grasp_startup.hpp"

namespace stonefish_net
{
namespace
{
constexpr const char* kMaterial = "ZipTieNylon";
constexpr sf::Scalar kGraspICStep = 0.001;

[[noreturn]] void invalid(const std::string& message)
{
  throw std::invalid_argument("zip_tie: " + message);
}

std::string attribute(const tinyxml2::XMLElement& element, const char* name)
{
  const char* value = element.Attribute(name);
  if (value == nullptr || *value == '\0') invalid(std::string("missing ") + name);
  return value;
}

double number(const tinyxml2::XMLElement& element, const char* name, double fallback)
{
  if (element.Attribute(name) == nullptr) return fallback;
  double result;
  if (element.QueryDoubleAttribute(name, &result) != tinyxml2::XML_SUCCESS || !std::isfinite(result)) {
    invalid(std::string("invalid ") + name);
  }
  return result;
}

sf::Vector3 vector(const std::string& value)
{
  std::istringstream stream(value);
  double x, y, z;
  std::string excess;
  if (!(stream >> x >> y >> z) || (stream >> excess) ||
      !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) invalid("expected three finite coordinates");
  return sf::Vector3(x, y, z);
}

sf::Vector3 vector(const YAML::Node& node)
{
  if (!node.IsSequence() || node.size() != 3) invalid("invalid asset coordinate");
  const double x = node[0].as<double>(), y = node[1].as<double>(), z = node[2].as<double>();
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) invalid("non-finite asset coordinate");
  return sf::Vector3(x, y, z);
}

sf::Transform translation(const sf::Vector3& value) { return sf::Transform(sf::IQ(), value); }

void synchronize(sf::FeatherstoneEntity& dynamics, sf::SimulationManager& sm)
{
  btAlignedObjectArray<sf::Quaternion> rotations;
  btAlignedObjectArray<sf::Vector3> positions;
  dynamics.getMultiBody()->forwardKinematics(rotations, positions);
  dynamics.getMultiBody()->updateCollisionObjectWorldTransforms(rotations, positions);
  sm.getDynamicsWorld()->updateAabbs();
}

unsigned int joint_index(sf::FeatherstoneEntity& dynamics, const sf::Servo& servo)
{
  const std::string name = servo.getJointName();
  for (unsigned int i = 0; i < dynamics.getNumOfJoints(); ++i) {
    if (dynamics.getJointName(i) == name) return i;
  }
  invalid("servo joint not found: " + name);
}

sf::Servo* servo(sf::Robot& robot, const std::string& name)
{
  auto* result = dynamic_cast<sf::Servo*>(robot.getActuator(robot.getName() + "/" + name));
  if (!result) invalid("servo not found: " + name);
  return result;
}

btCollisionObject* collider(sf::SolidEntity& solid)
{
  if (solid.getRigidBody()) return solid.getRigidBody();
  return solid.getMultiBodyLinkCollider();
}

// Also reports small positive distances: initialization stops just outside contact.
struct Distance : btCollisionWorld::ContactResultCallback
{
  explicit Distance(double threshold) { m_closestDistanceThreshold = threshold; }
  btScalar minimum = BT_LARGE_FLOAT;
  btScalar addSingleResult(btManifoldPoint& point, const btCollisionObjectWrapper*, int, int,
                           const btCollisionObjectWrapper*, int, int) override
  {
    minimum = btMin(minimum, point.getDistance());
    return 0;
  }
};

double distance(sf::SimulationManager& sm, sf::SolidEntity& a, sf::SolidEntity& b, double threshold)
{
  Distance result(threshold);
  sm.getDynamicsWorld()->contactPairTest(collider(a), collider(b), result);
  return result.minimum;
}

double fit_finger(sf::SimulationManager& sm, sf::FeatherstoneEntity& dynamics, unsigned int joint,
                  sf::SolidEntity& finger, sf::SolidEntity& support, double open, double closed, double gap)
{
  const auto definition = dynamics.getJoint(joint);
  if (open < definition.lowerLimit || open > definition.upperLimit ||
      closed < definition.lowerLimit || closed > definition.upperLimit) invalid("grasp angles exceed joint limits");
  auto separation = [&](double angle) {
    dynamics.setJointIC(joint, angle, 0);
    synchronize(dynamics, sm);
    return distance(sm, finger, support, 2 * gap);
  };
  if (separation(open) < gap) invalid("support intersects an open finger; adjust grasp position/open angles");
  if (separation(closed) > gap) invalid("finger cannot reach support; adjust grasp position/closed angles");
  double safe = open, touching = closed;
  for (int i = 0; i < 28; ++i) {
    const double middle = (safe + touching) / 2;
    if (separation(middle) >= gap) safe = middle;
    else touching = middle;
  }
  dynamics.setJointIC(joint, safe, 0);
  synchronize(dynamics, sm);
  return safe;
}

sf::Scalar inertia_at(const sf::SolidEntity& body, const sf::Vector3& pivot, const sf::Vector3& axis)
{
  const sf::Transform cg = body.getCGTransform();
  const sf::Vector3 local = cg.getBasis().transpose() * axis;
  const sf::Vector3 inertia = body.getAugmentedInertia();
  const sf::Vector3 offset = cg.getOrigin() - pivot;
  return inertia.dot(local * local) + body.getAugmentedMass() *
    btMax(sf::Scalar(0), offset.length2() - btPow(offset.dot(axis), 2));
}

// Implicit Kelvin-Voigt angular springs. Bullet's explicit Spring2 safety limiter
// would clamp all three EI/L values to nearly the same small stiffness at 200 Hz,
// losing the ribbon's resistance to bending across its width. Soft angular rows
// preserve that anisotropy, with ERP/CFM recalculated for the actual solver step.
class RibbonConstraint : public btGeneric6DofSpring2Constraint
{
public:
  RibbonConstraint(sf::SolidEntity& a, sf::SolidEntity& b, const sf::Transform& frame,
                   const sf::Vector3& stiffness, const sf::Vector3& damping)
  : btGeneric6DofSpring2Constraint(*a.getRigidBody(), *b.getRigidBody(),
      a.getCGTransform().inverse() * frame, b.getCGTransform().inverse() * frame, RO_XYZ),
    stiffness_(stiffness), damping_(damping)
  {
    setLinearLowerLimit(sf::V0());
    setLinearUpperLimit(sf::V0());
    // Zero rest angle with finite compliance, rather than a rigid angular lock.
    setAngularLowerLimit(sf::V0());
    setAngularUpperLimit(sf::V0());
    setOverrideNumSolverIterations(120);
  }

  void getInfo2(btConstraintInfo2* info) override
  {
    // Stonefish sets global joint damping to zero. These implicit rows need the
    // usual relative-velocity term as well as the position correction.
    info->m_damping = 1;
    const sf::Scalar dt = 1 / info->fps;
    for (int i = 0; i < 3; ++i) {
      const sf::Scalar denominator = dt * stiffness_[i] + damping_[i];
      setParam(BT_CONSTRAINT_STOP_ERP, dt * stiffness_[i] / denominator, i + 3);
      setParam(BT_CONSTRAINT_STOP_CFM, 0, i + 3);
    }
    btGeneric6DofSpring2Constraint::getInfo2(info);
    // In RO_XYZ the first three rows are the x/y/z angular rows. Bullet's joint
    // solver expects dimensionless CFM (its row diagonal is not adjusted for it).
    for (int i = 0; i < 3; ++i) {
      const sf::Vector3 axis = getAxis(i);
      const sf::Scalar diagonal = axis.dot((m_rbA.getInvInertiaTensorWorld() +
                                            m_rbB.getInvInertiaTensorWorld()) * axis);
      const sf::Scalar gamma = 1 / (dt * (dt * stiffness_[i] + damping_[i]));
      // Extremely soft user settings must not destabilize the iterative solver.
      info->cfm[i * info->rowskip] = btMin(sf::Scalar(0.9), gamma / diagonal);
    }
  }

private:
  sf::Vector3 stiffness_, damping_;
};

class RibbonJoint : public sf::Joint
{
public:
  RibbonJoint(const std::string& name, sf::SolidEntity& a, sf::SolidEntity& b,
              const sf::Transform& frame, const sf::Vector3& stiffness, const sf::Vector3& damping)
  : sf::Joint(name, false)
  {
    jSolidA = &a;
    jSolidB = &b;
    setConstraint(new RibbonConstraint(a, b, frame, stiffness, damping));
  }
  sf::JointType getType() const override { return sf::JointType::SPRING; }
};

void connect(sf::SimulationManager& sm, sf::SolidEntity& a, sf::SolidEntity& b,
             const sf::Transform& frame, const sf::Vector3& stiffness, double damping, size_t index)
{
  sf::Vector3 coefficients = sf::V0();
  for (int i = 0; i < 3; ++i) {
    const sf::Vector3 axis = frame.getBasis().getColumn(i);
    const double ia = inertia_at(a, frame.getOrigin(), axis);
    const double ib = inertia_at(b, frame.getOrigin(), axis);
    const double effective = ia * ib / (ia + ib);
    coefficients[i] = 2 * damping * std::sqrt(stiffness[i] * effective);
  }
  sm.AddJoint(new RibbonJoint("zip_tie_joint_" + std::to_string(index), a, b, frame, stiffness, coefficients));
}

// SolveICProblem normally finishes before actuators get their first Update().
// Participate in its existing IC joint hook to close the fingers under zero
// gravity and try to establish a real, settled two-finger grasp. A timeout
// releases the initialization gate too, without claiming a successful grasp.
// This object has no Bullet constraint: it never attaches the zip tie to the
// palm or the world.
class InitialGrasp : public sf::Joint
{
public:
  InitialGrasp(sf::SimulationManager& sm, sf::SolidEntity& support, sf::SolidEntity& palm,
               const std::array<sf::SolidEntity*, 2>& jaws, const std::array<sf::Servo*, 2>& servos,
               const std::array<btMultiBodyJointMotor*, 2>& motors, sf::Scalar torque)
  : sf::Joint("zip_tie_initial_grasp"), sm_(sm), support_(support), palm_(palm), jaws_(jaws),
    servos_(servos), motors_(motors), torque_(torque)
  {
  }

  sf::JointType getType() const override { return sf::JointType::SPRING; }

  bool SolvePositionIC(sf::Scalar, sf::Scalar) override
  {
    // Both success and timeout are terminal: do not restart closure, repeat the
    // warning or reapply IC-step motor impulses on subsequent solver calls.
    if (startup_.state() != core::GraspStartup::State::Closing) return true;
    for (size_t i = 0; i < servos_.size(); ++i) {
      servos_[i]->Update(kGraspICStep);
      // Stonefish's setter scales torque by the normal 200 Hz step. During IC
      // the motor impulse must instead use the 1 ms initialization step.
      motors_[i]->setMaxAppliedImpulse(torque_ * kGraspICStep);
    }

    std::array<double, 2> forces{};
    auto* dispatcher = sm_.getDynamicsWorld()->getDispatcher();
    for (int m = 0; m < dispatcher->getNumManifolds(); ++m) {
      const auto* manifold = dispatcher->getManifoldByIndexInternal(m);
      for (size_t i = 0; i < jaws_.size(); ++i) {
        const auto* jaw = collider(*jaws_[i]);
        if (!((manifold->getBody0() == support_.getRigidBody() && manifold->getBody1() == jaw) ||
              (manifold->getBody1() == support_.getRigidBody() && manifold->getBody0() == jaw))) continue;
        for (int c = 0; c < manifold->getNumContacts(); ++c) {
          const auto& point = manifold->getContactPoint(c);
          // Bullet may retain a loaded contact with a tiny positive distance.
          // Require its solved normal impulse, rather than exact overlap.
          if (point.getDistance() <= sf::Scalar(1e-4)) forces[i] += point.getAppliedImpulse() / kGraspICStep;
        }
      }
    }
    const sf::Vector3 offset = palm_.getCGTransform().inverse() * support_.getCGTransform().getOrigin();
    const bool steady = (support_.getLinearVelocity() - palm_.getLinearVelocityInLocalPoint(offset)).length() < 0.01 &&
      (support_.getAngularVelocity() - palm_.getAngularVelocity()).length() < 0.1;
    const auto state = startup_.update(kGraspICStep, forces, steady);
    if (state == core::GraspStartup::State::Closing) return false;

    for (auto* servo : servos_) servo->setMaxTorque(torque_);  // Restore the normal-step motor impulse.
    if (state == core::GraspStartup::State::Failed) {
      std::clog << "[zip_tie] ADVERTENCIA: agarre inicial no confirmado tras 2 s; "
                   "la simulacion continua con fisica normal. La brida puede quedar libre. "
                   "Fuerzas por dedo: " << forces[0] << ", " << forces[1]
                << " N; conjunto estable: " << (steady ? "si" : "no") << "." << std::endl;
    } else {
      std::clog << "[zip_tie] Agarre inicial confirmado: ambos dedos presionan el soporte ("
                << forces[0] << ", " << forces[1] << " N)." << std::endl;
    }
    return true;
  }

private:
  sf::SimulationManager& sm_;
  sf::SolidEntity& support_;
  sf::SolidEntity& palm_;
  std::array<sf::SolidEntity*, 2> jaws_;
  std::array<sf::Servo*, 2> servos_;
  std::array<btMultiBodyJointMotor*, 2> motors_;
  sf::Scalar torque_;
  core::GraspStartup startup_;
};

}  // namespace

ZipTie::ZipTie() = default;
ZipTie::~ZipTie() = default;

std::unique_ptr<ZipTie> ZipTie::Create(sf::SimulationManager& sm, const std::string& scenario)
{
  tinyxml2::XMLDocument document;
  if (document.LoadFile(scenario.c_str()) != tinyxml2::XML_SUCCESS) invalid("cannot read scenario " + scenario);
  const auto* root = document.FirstChildElement("scenario");
  if (!root) invalid("missing scenario root");
  const auto* config = root->FirstChildElement("zip_tie");
  if (!config) return nullptr;
  if (config->NextSiblingElement("zip_tie")) invalid("only one zip_tie per scene is supported");

  const double young = number(*config, "young_modulus", 1e9);
  const double stiffness_scale = number(*config, "stiffness_scale", 1.0);
  const double thin_bending_scale = number(*config, "thin_bending_scale", 1.0);
  if (thin_bending_scale <= 0) invalid("thin_bending_scale must be positive");
  const double effective_young = young * stiffness_scale;
  const double poisson = number(*config, "poisson_ratio", 0.4);
  const double damping = number(*config, "damping_ratio", 0.35);
  const double density = number(*config, "density", 1140);
  const double friction = number(*config, "static_friction", 0.6);
  const double dynamic_friction = number(*config, "dynamic_friction", 0.4);
  const double torque = number(*config, "grasp_torque", 0.1);
  const double gap = number(*config, "grasp_clearance", 0.00005);
  const double preload = number(*config, "grasp_preload", 0.015);
  if (young <= 0 || stiffness_scale <= 0 || effective_young <= 0 || !std::isfinite(effective_young) ||
      poisson < 0 || poisson >= 0.5 || damping < 0 || damping > 1 || density <= 0 ||
      friction < 0 || dynamic_friction < 0 || dynamic_friction > friction || torque <= 0 ||
      gap <= 0 || gap > 0.001 || preload <= 0 || preload > 0.1) invalid("physical parameters outside valid range");

  int requested_segments = 0;  // Old scenes may leave the count in asset.json.
  if (config->Attribute("segments") &&
      (config->QueryIntAttribute("segments", &requested_segments) != tinyxml2::XML_SUCCESS ||
       requested_segments < 2 || requested_segments > 64)) invalid("segments must be an integer from 2 to 64");
  std::string asset_name = attribute(*config, "asset");
  const std::string token = "{segments}";
  const auto marker = asset_name.find(token);
  if (marker != std::string::npos) {
    if (requested_segments == 0) invalid("segments is required for an asset path containing {segments}");
    asset_name.replace(marker, token.size(), std::to_string(requested_segments));
  }
  const std::filesystem::path asset_path = std::filesystem::path(sf::SimulationApp::getApp()->getDataPath()) /
    asset_name;
  if (!std::filesystem::is_regular_file(asset_path)) {
    invalid("missing asset " + asset_path.string() + "; generate this variant with prepare_zip_tie.py --segments N");
  }
  const auto asset = YAML::LoadFile(asset_path.string());
  const auto pieces = asset["segments"];
  if (!pieces.IsSequence() || pieces.size() < 2 || pieces.size() > 64) {
    invalid("asset must contain between 2 and 64 segments");
  }
  if (requested_segments != 0 && pieces.size() != static_cast<std::size_t>(requested_segments)) {
    invalid("segments does not match the selected asset");
  }
  const double length = asset["length"].as<double>();
  const double width = asset["width"].as<double>(), thickness = asset["thickness"].as<double>();
  if (!std::isfinite(length) || !std::isfinite(width) || !std::isfinite(thickness) ||
      length <= 0 || width <= 0 || thickness <= 0 || thickness > width) invalid("invalid ribbon dimensions");
  sf::Vector3 grasp_point = vector(asset["support"]["origin"]);
  std::size_t grasp_body_index = 0;  // Rigid support/head for existing scenes.
  if (config->Attribute("grasp_from_tip")) {
    const double from_tip = number(*config, "grasp_from_tip", 0);
    if (from_tip <= 0 || from_tip >= length) invalid("grasp_from_tip must lie inside the flexible strip");
    grasp_point = vector(asset["root"]);
    grasp_point.setZ(grasp_point.z() - length + from_tip);
    const auto segment = static_cast<std::size_t>((length - from_tip) / (length / pieces.size()));
    grasp_body_index = 1 + std::min(segment, pieces.size() - 1);
  }
  // Resolve and validate every mesh before changing the robot or adding bodies.
  for (const auto& piece : {asset["support"], asset["head"]}) {
    vector(piece["origin"]);
    for (const char* key : {"visual", "physical"}) {
      if (!std::filesystem::is_regular_file(asset_path.parent_path() / piece[key].as<std::string>())) invalid("missing mesh");
    }
  }
  for (const auto& piece : pieces) {
    vector(piece["origin"]);
    vector(piece["joint"]);
    for (const char* key : {"visual", "physical"}) {
      if (!std::filesystem::is_regular_file(asset_path.parent_path() / piece[key].as<std::string>())) invalid("missing mesh");
    }
  }

  auto result = std::unique_ptr<ZipTie>(new ZipTie);
  result->segment_count_ = pieces.size();
  result->robot_ = attribute(*config, "robot");
  auto* robot = dynamic_cast<sf::FeatherstoneRobot*>(sm.getRobot(result->robot_));
  if (!robot) invalid("expected a Featherstone robot " + result->robot_);
  auto* dynamics = robot->getDynamics();
  auto* palm = robot->getLink(result->robot_ + "/" + attribute(*config, "palm"));
  if (!palm) invalid("grasp palm not found");
  const sf::Vector3 position = vector(attribute(*config, "grasp_xyz"));
  const sf::Vector3 rpy = vector(attribute(*config, "grasp_rpy"));
  sf::Quaternion rotation;
  rotation.setEulerZYX(rpy.z(), rpy.y(), rpy.x());

  std::array<sf::SolidEntity*, 2> jaws;
  std::array<unsigned int, 2> joints;
  std::array<double, 2> open, closed;
  for (size_t i = 0; i < 2; ++i) {
    const char* suffix = i == 0 ? "a" : "b";
    result->fingers_[i].servo = servo(*robot, attribute(*config, (std::string("servo_") + suffix).c_str()));
    joints[i] = joint_index(*dynamics, *result->fingers_[i].servo);
    jaws[i] = robot->getLink(result->robot_ + "/" + attribute(*config, (std::string("finger_") + suffix).c_str()));
    if (!jaws[i] || !collider(*jaws[i])) invalid("grasp finger has no collider");
    open[i] = number(*config, (std::string("open_") + suffix).c_str(), i == 0 ? -0.5 : 0.5);
    closed[i] = number(*config, (std::string("closed_") + suffix).c_str(), i == 0 ? 0.125 : -0.125);
    if (open[i] == closed[i]) invalid("open and closed angles must differ");
    const auto definition = dynamics->getJoint(joints[i]);
    if (!definition.motor) invalid("finger has no joint motor");
    result->fingers_[i].original_torque = definition.motor->getMaxAppliedImpulse() * sm.getStepsPerSecond();
  }
  // Start the right arm at the positions already requested by its scene servos.
  // Other scenes never execute this code, and the left arm is not changed.
  for (int i = 1; i <= 6; ++i) {
    auto* motor = servo(*robot, "Servo" + std::to_string(i) + "R");
    dynamics->setJointIC(joint_index(*dynamics, *motor), motor->getDesiredPosition(), 0);
    motor->Update(1 / sm.getStepsPerSecond());
  }
  for (size_t i = 0; i < 2; ++i) dynamics->setJointIC(joints[i], open[i], 0);
  synchronize(*dynamics, sm);

  auto* materials = sm.getMaterialManager();
  const auto existing = materials->GetMaterialsList();
  if (std::find(existing.begin(), existing.end(), kMaterial) != existing.end()) invalid("material name already in use");
  materials->CreateMaterial(kMaterial, density, 0);
  for (const auto& name : materials->GetMaterialsList()) {
    materials->SetMaterialsInteraction(kMaterial, name, friction, dynamic_friction);
  }
  sf::PhysicsSettings physics;  // SUBMERGED, collisions and buoyancy enabled.
  auto make_piece = [&](const std::string& name, const YAML::Node& piece, const char* look) {
    return new sf::Polyhedron(name, physics,
      (asset_path.parent_path() / piece["visual"].as<std::string>()).string(), 1, sf::I4(),
      (asset_path.parent_path() / piece["physical"].as<std::string>()).string(), 1, sf::I4(), kMaterial, look);
  };
  auto* support = make_piece("zip_tie_support", asset["support"], "zip_tie_red");
  auto* head = make_piece("zip_tie_head", asset["head"], "zip_tie_white");
  auto* rigid = new sf::Compound("zip_tie_rigid", physics, support, translation(vector(asset["support"]["origin"])));
  rigid->AddExternalPart(head, translation(vector(asset["head"]["origin"])));
  const sf::Transform pose = palm->getOTransform() * sf::Transform(rotation,
    position - quatRotate(rotation, grasp_point));
  sm.AddSolidEntity(rigid, pose);
  rigid->getRigidBody()->setActivationState(DISABLE_DEACTIVATION);

  const double step = length / pieces.size();
  const double thin_inertia = width * std::pow(thickness, 3) / 12;
  const double wide_inertia = thickness * std::pow(width, 3) / 12;
  const double ratio = thickness / width;
  const double torsion = width * std::pow(thickness, 3) *
    (1.0 / 3 - 0.21 * ratio * (1 - std::pow(ratio, 4) / 12));
  // Scale only bending through the thin dimension; wide bending and torsion
  // retain their material/geometric stiffness. connect() keeps the damping ratio.
  const double thin_stiffness = effective_young * thin_inertia / step * thin_bending_scale;
  if (!std::isfinite(thin_stiffness) || thin_stiffness <= 0) {
    invalid("thin_bending_scale produces invalid bending stiffness");
  }
  const sf::Vector3 stiffness(thin_stiffness, effective_young * wide_inertia / step,
                             effective_young / (2 * (1 + poisson)) * torsion / step);
  std::vector<sf::SolidEntity*> bodies{rigid};
  sf::SolidEntity* previous = rigid;
  for (size_t i = 0; i < pieces.size(); ++i) {
    auto* piece = make_piece("zip_tie_strip_" + std::to_string(i), pieces[i], "zip_tie_white");
    sm.AddSolidEntity(piece, pose * translation(vector(pieces[i]["origin"])));
    piece->getRigidBody()->setActivationState(DISABLE_DEACTIVATION);
    connect(sm, *previous, *piece, pose * translation(vector(pieces[i]["joint"])), stiffness, damping, i);
    bodies.push_back(piece);
    previous = piece;
  }
  auto* grasp_body = bodies[grasp_body_index];

  for (size_t i = 0; i < 2; ++i) {
    const double safe = fit_finger(sm, *dynamics, joints[i], *jaws[i], *grasp_body, open[i], closed[i], gap);
    auto& finger = result->fingers_[i];
    const auto definition = dynamics->getJoint(joints[i]);
    finger.servo->setMaxTorque(torque);
    finger.servo->setDesiredVelocity(0);
    finger.servo->setControlMode(sf::ServoControlMode::POSITION);
    finger.servo->setDesiredPosition(safe);
    finger.target = btClamped(sf::Scalar(safe + (closed[i] > open[i] ? preload : -preload)),
                              definition.lowerLimit, definition.upperLimit);
    finger.servo->setDesiredPosition(finger.target);
    finger.servo->Update(kGraspICStep);  // Prime the closure before the very first IC solver step.
    definition.motor->setMaxAppliedImpulse(torque * kGraspICStep);
  }
  // Check all robot links, not just the fingertips, before accepting the pose.
  for (auto* body : bodies) {
    for (unsigned int r = 0; sf::Robot* other = sm.getRobot(r); ++r) {
      for (size_t i = 0; sf::SolidEntity* link = other->getLink(i); ++i) {
        if (collider(*link) && distance(sm, *body, *link, 0) < -1e-5) {
          invalid("initial penetration with " + link->getName() + "; adjust grasp_xyz/grasp_rpy");
        }
      }
    }
  }
  // No weight or hydrodynamic forces until the initial grasp is loaded. Other
  // scenes retain their IC configuration because this path requires <zip_tie>.
  sm.setICSolverParams(false, kGraspICStep);
  result->grasp_ = ZipTieGrasp::Create(sm, *robot, *config, bodies, *grasp_body);
  if (result->grasp_ && result->grasp_->InitiallyAttached()) {
    // The software attachment is already active before the first IC/dynamics
    // step. A contact-force gate would never succeed with these pairs ignored.
    for (std::size_t i = 0; i < 2; ++i) {
      dynamics->setJointIC(joints[i], result->fingers_[i].target, 0);
      result->fingers_[i].servo->setMaxTorque(torque);  // Restore the normal-step impulse.
    }
    synchronize(*dynamics, sm);
    std::clog << "[zip_tie] Agarre inicial por software activado antes de iniciar la dinamica." << std::endl;
  } else {
    sm.AddJoint(new InitialGrasp(sm, *grasp_body, *palm, jaws,
      {result->fingers_[0].servo, result->fingers_[1].servo},
      {dynamics->getJoint(joints[0]).motor, dynamics->getJoint(joints[1]).motor}, torque));
  }
  return result;
}

void ZipTie::InitializeCommands(Commands& commands) const
{
  for (const auto& finger : fingers_) {
    commands[finger.servo->getJointName()] = {sf::ServoControlMode::POSITION, finger.target};
  }
}

void ZipTie::HandleCommands(const Commands& commands)
{
  for (auto& finger : fingers_) {
    const auto command = commands.find(finger.servo->getJointName());
    if (finger.initial_grasp && command != commands.end() &&
        (command->second.first != sf::ServoControlMode::POSITION ||
         btFabs(command->second.second - finger.target) > 1e-6)) {
      finger.servo->setMaxTorque(finger.original_torque);
      finger.initial_grasp = false;
    }
  }
}

void ZipTie::UpdateGrasp(const Commands& commands)
{
  if (grasp_) grasp_->Update(commands);
}

}  // namespace stonefish_net
