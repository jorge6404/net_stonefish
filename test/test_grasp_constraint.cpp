#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <BulletCollision/BroadphaseCollision/btDbvtBroadphase.h>
#include <BulletCollision/CollisionDispatch/btCollisionDispatcher.h>
#include <BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h>
#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>
#include <BulletDynamics/Featherstone/btMultiBodyLinkCollider.h>
#include <LinearMath/btDefaultMotionState.h>

#include "stonefish_net/GraspConstraint.h"

namespace
{
using stonefish_net::GraspConstraint;
constexpr double kTolerance = sizeof(btScalar) == sizeof(double) ? 1e-10 : 1e-5;

void expectSameFrame(const btMultiBody& robot, int link, const btRigidBody& piece,
                     const GraspConstraint& constraint)
{
  const auto pivot = robot.localPosToWorld(link, constraint.getPivotInA());
  const auto basis = robot.localFrameToWorld(link, constraint.getFrameInA());
  const auto& physical = piece.getCenterOfMassTransform();
  EXPECT_NEAR((pivot - physical.getOrigin()).length(), 0, kTolerance);
  for (int axis = 0; axis < 3; ++axis) {
    EXPECT_NEAR((basis.getColumn(axis) - physical.getBasis().getColumn(axis)).length(), 0, kTolerance);
  }
}

TEST(GraspConstraint, UsesThePhysicsPoseWhenTheMotionStateHasNotBeenSynchronized)
{
  btMultiBody robot(1, 20, btVector3(1, 2, 3), false, false);
  robot.setupRevolute(0, 1, btVector3(0.1, 0.2, 0.3), -1,
    btQuaternion(btVector3(1, 1, 0).normalized(), 0.3), btVector3(0, 1, 0),
    btVector3(0.4, 0, 0.2), btVector3(0, 0, 0.1));
  robot.finalizeMultiDof();
  robot.setBaseWorldTransform(btTransform(btQuaternion(btVector3(0, 0, 1), 0.8), btVector3(1, 2, 3)));
  robot.setJointPos(0, -0.6);
  btAlignedObjectArray<btQuaternion> rotations;
  btAlignedObjectArray<btVector3> positions;
  robot.forwardKinematics(rotations, positions);

  const btTransform old_pose(btQuaternion::getIdentity(), btVector3(-20, 30, 10));
  btDefaultMotionState motion_state(old_pose);
  btBoxShape shape(btVector3(0.01, 0.02, 0.03));
  btRigidBody piece(0.02, &motion_state, &shape, btVector3(1e-5, 2e-5, 3e-5));
  for (int link : {-1, 0}) {
    for (double angle : {0.0, 0.4, 1.8, -2.5}) {
      const btTransform physical(btQuaternion(btVector3(1, -2, 3).normalized(), angle), btVector3(2, -1, 4));
      piece.setCenterOfMassTransform(physical);
      btTransform interpolated;
      motion_state.getWorldTransform(interpolated);
      ASSERT_GT((interpolated.getOrigin() - physical.getOrigin()).length(), 1);

      GraspConstraint grasp(robot, link, piece, 50, 2, 0.1);
      expectSameFrame(robot, link, piece, grasp);
    }
  }
}

class RecordedGrasp : public GraspConstraint
{
public:
  using GraspConstraint::GraspConstraint;
  struct Row { btScalar lower, upper, rhs, diagonal; };
  std::array<Row, 6> recorded{};
  int evaluations = 0;

  void createConstraintRows(btMultiBodyConstraintArray& rows, btMultiBodyJacobianData& data,
                            const btContactSolverInfo& info) override
  {
    const int first = rows.size();
    GraspConstraint::createConstraintRows(rows, data, info);
    for (int i = 0; i < 6; ++i) {
      const auto& row = rows[first + i];
      recorded[i] = {row.m_lowerLimit, row.m_upperLimit, row.m_rhs, row.m_jacDiagABInv};
    }
    ++evaluations;
  }
};

struct World
{
  btDefaultCollisionConfiguration configuration;
  btCollisionDispatcher dispatcher{&configuration};
  btDbvtBroadphase broadphase;
  btMultiBodyConstraintSolver solver;
  btMultiBodyDynamicsWorld dynamics{&dispatcher, &broadphase, &solver, &configuration};
  btMultiBody robot{0, 20, btVector3(1, 2, 3), false, false};
  btBoxShape base_shape{btVector3(0.1, 0.1, 0.1)};
  btMultiBodyLinkCollider base_collider{&robot, -1};
  btBoxShape shape{btVector3(0.01, 0.02, 0.03)};
  btDefaultMotionState motion_state{btTransform(btQuaternion::getIdentity(), btVector3(-20, 30, 10))};
  btRigidBody piece{0.02, &motion_state, &shape, btVector3(1e-5, 2e-5, 3e-5)};

  World()
  {
    dynamics.setGravity(btVector3(0, 0, 0));
    robot.finalizeMultiDof();
    robot.setBaseWorldTransform(btTransform(btQuaternion(btVector3(0, 0, 1), 0.8), btVector3(1, 2, 3)));
    base_collider.setCollisionShape(&base_shape);
    base_collider.setWorldTransform(robot.getBaseWorldTransform());
    robot.setBaseCollider(&base_collider);
    dynamics.addMultiBody(&robot);
    dynamics.addCollisionObject(&base_collider, 1, 0);
    piece.setCenterOfMassTransform(btTransform(btQuaternion(btVector3(1, 2, 3).normalized(), 1.1),
                                              btVector3(1.3, 2.2, 3.1)));
    piece.setActivationState(DISABLE_DEACTIVATION);
    dynamics.addRigidBody(&piece, 1, 0);
  }

  ~World()
  {
    dynamics.removeRigidBody(&piece);
    dynamics.removeCollisionObject(&base_collider);
    dynamics.removeMultiBody(&robot);
  }
};

TEST(GraspConstraint, AnUnsynchronizedMotionStateDoesNotKickTheRobotOnAttachment)
{
  World world;
  const btTransform base = world.robot.getBaseWorldTransform();
  const btTransform piece = world.piece.getCenterOfMassTransform();
  RecordedGrasp grasp(world.robot, -1, world.piece, 50, 2, 0.1);
  world.dynamics.addMultiBodyConstraint(&grasp);
  for (int i = 0; i < 200; ++i) world.dynamics.stepSimulation(0.005, 0);
  ASSERT_GT(grasp.evaluations, 0);
  EXPECT_NEAR((world.robot.getBaseWorldTransform().getOrigin() - base.getOrigin()).length(), 0, kTolerance);
  EXPECT_NEAR((world.piece.getCenterOfMassTransform().getOrigin() - piece.getOrigin()).length(), 0, kTolerance);
  EXPECT_NEAR(world.robot.getBaseVel().length(), 0, kTolerance);
  EXPECT_NEAR(world.robot.getBaseOmega().length(), 0, kTolerance);
  world.dynamics.removeMultiBodyConstraint(&grasp);
}

TEST(GraspConstraint, ForceAndTorqueLimitsScaleWithTheActualStep)
{
  for (double dt : {0.001, 0.005}) {
    World world;
    RecordedGrasp grasp(world.robot, -1, world.piece, 50, 2, 0.1);
    world.dynamics.addMultiBodyConstraint(&grasp);
    // Deliberately demand a much larger correction than the allowed impulse.
    world.piece.setLinearVelocity(btVector3(100, -200, 300));
    world.piece.setAngularVelocity(btVector3(100, -200, 300));
    world.dynamics.stepSimulation(dt, 0);
    ASSERT_GT(grasp.evaluations, 0);
    for (int i = 0; i < 6; ++i) {
      const double limit = (i < 3 ? 50 : 2) * dt;
      EXPECT_NEAR(grasp.recorded[i].lower, -limit, kTolerance);
      EXPECT_NEAR(grasp.recorded[i].upper, limit, kTolerance);
      EXPECT_LE(std::abs(grasp.getAppliedImpulse(i)), limit + kTolerance);
      EXPECT_TRUE(std::isfinite(grasp.recorded[i].rhs));
    }
    world.dynamics.removeMultiBodyConstraint(&grasp);
  }
}

TEST(GraspConstraint, UsesItsOwnErpWithoutChangingTheWorld)
{
  World world;
  world.dynamics.getSolverInfo().m_erp = 0.8;
  RecordedGrasp grasp(world.robot, -1, world.piece, 50, 2, 0.1);
  world.dynamics.addMultiBodyConstraint(&grasp);
  auto shifted = world.piece.getCenterOfMassTransform();
  shifted.getOrigin().setX(shifted.getOrigin().x() + 0.05);
  world.piece.setCenterOfMassTransform(shifted);
  world.dynamics.stepSimulation(0.005, 0);
  ASSERT_GT(grasp.evaluations, 0);
  ASSERT_GT(grasp.recorded[0].diagonal, 0);
  EXPECT_NEAR(grasp.recorded[0].rhs / grasp.recorded[0].diagonal, 0.05 * 0.1 / 0.005, kTolerance);
  EXPECT_NEAR(world.dynamics.getSolverInfo().m_erp, 0.8, kTolerance);
  world.dynamics.removeMultiBodyConstraint(&grasp);
}
}  // namespace
