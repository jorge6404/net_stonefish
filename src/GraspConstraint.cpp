#include "stonefish_net/GraspConstraint.h"

#include <algorithm>
#include <BulletDynamics/Dynamics/btRigidBody.h>

namespace stonefish_net
{
namespace
{
btMatrix3x3 frameInRobot(const btMultiBody& robot, int link, const btRigidBody& piece)
{
  const auto& basis = piece.getCenterOfMassTransform().getBasis();
  const auto x = robot.worldDirToLocal(link, basis.getColumn(0));
  const auto y = robot.worldDirToLocal(link, basis.getColumn(1));
  const auto z = robot.worldDirToLocal(link, basis.getColumn(2));
  return btMatrix3x3(x.x(), y.x(), z.x(), x.y(), y.y(), z.y(), x.z(), y.z(), z.z());
}
}  // namespace

GraspConstraint::GraspConstraint(btMultiBody& robot, int link, btRigidBody& piece,
                               double max_force, double max_torque, double erp)
  : btMultiBodyFixedConstraint(&robot, link, &piece,
      robot.worldPosToLocal(link, piece.getCenterOfMassTransform().getOrigin()),
      btVector3(0, 0, 0), frameInRobot(robot, link, piece), btMatrix3x3::getIdentity()),
    max_force_(max_force), max_torque_(max_torque), erp_(erp)
{
  for (int row = 0; row < getNumRows(); ++row) internalSetAppliedImpulse(row, 0);
}

void GraspConstraint::createConstraintRows(btMultiBodyConstraintArray& rows, btMultiBodyJacobianData& data,
                                         const btContactSolverInfo& info)
{
  // btMultiBodyFixedConstraint's inherited setErp() is a no-op. Supply the
  // grasp's ERP explicitly without changing the world's other constraints.
  btContactSolverInfo local = info;
  local.m_erp = erp_;
  const btScalar force_impulse = max_force_ * info.m_timeStep;
  const btScalar torque_impulse = max_torque_ * info.m_timeStep;
  setMaxAppliedImpulse(std::max(force_impulse, torque_impulse));
  const int first = rows.size();
  btMultiBodyFixedConstraint::createConstraintRows(rows, data, local);
  for (int row = first; row < rows.size(); ++row) {
    const btScalar limit = row - first < 3 ? force_impulse : torque_impulse;
    rows[row].m_lowerLimit = -limit;
    rows[row].m_upperLimit = limit;
  }
}
}  // namespace stonefish_net
