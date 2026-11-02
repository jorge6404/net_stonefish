#pragma once

#include <BulletDynamics/Featherstone/btMultiBodyFixedConstraint.h>

namespace stonefish_net
{
// A fixed grasp uses the current physics pose, including when it is created
// inside a post-step callback before Bullet synchronizes the motion state.
class GraspConstraint : public btMultiBodyFixedConstraint
{
public:
  GraspConstraint(btMultiBody& robot, int link, btRigidBody& piece,
                  double max_force, double max_torque, double erp);

  void createConstraintRows(btMultiBodyConstraintArray& rows, btMultiBodyJacobianData& data,
                            const btContactSolverInfo& info) override;

private:
  btScalar max_force_, max_torque_, erp_;
};
}  // namespace stonefish_net
