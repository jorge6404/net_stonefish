#include "stonefish_net/SolidContacts.h"

#include <utility>

#include <BulletCollision/CollisionShapes/btCompoundShape.h>
#include <BulletCollision/CollisionShapes/btConvexShape.h>
#include <BulletDynamics/Featherstone/btMultiBodyLinkCollider.h>
#include <Stonefish/entities/SolidEntity.h>

#include "convert.h"
#include "pose_now.h"

namespace stonefish_net
{
namespace
{
// Amplía [lo, hi] con la forma `shape` colocada con `t` (marco de la forma → marco de la caja). Las convexas
// van por puntos soporte (caja exacta aunque estén giradas); las compuestas, hija a hija; el resto (mallas
// cóncavas), con su AABB de Bullet.
void grow_aabb(const btCollisionShape& shape, const btTransform& t, btVector3& lo, btVector3& hi)
{
  if (shape.isCompound()) {
    const auto& compound = static_cast<const btCompoundShape&>(shape);
    for (int k = 0; k < compound.getNumChildShapes(); ++k) {
      grow_aabb(*compound.getChildShape(k), t * compound.getChildTransform(k), lo, hi);
    }
    return;
  }
  if (shape.isConvex()) {
    const auto& convex = static_cast<const btConvexShape&>(shape);
    for (int axis = 0; axis < 3; ++axis) {
      for (const btScalar s : {btScalar(-1), btScalar(1)}) {
        btVector3 dir(0, 0, 0);
        dir[axis] = s;
        const btVector3 p = t * convex.localGetSupportingVertex(t.getBasis().transpose() * dir);
        lo.setMin(p);
        hi.setMax(p);
      }
    }
    return;
  }
  btVector3 a, b;
  shape.getAabb(t, a, b);
  lo.setMin(a);
  hi.setMax(b);
}
}  // namespace

std::optional<core::RoundedBox> SolidContacts::fit_box(sf::SolidEntity& solid, double rounding_fraction)
{
  const btCollisionObject* co = solid.getRigidBody();
  if (co == nullptr) {
    co = solid.getMultiBodyLinkCollider();
  }
  if (co == nullptr || co->getCollisionShape() == nullptr) {
    return std::nullopt;
  }
  // La forma está en el marco CG del sólido y la caja va en su marco O (O = CG · T_CG2O)
  const btScalar inf = BT_LARGE_FLOAT;
  btVector3 lo(inf, inf, inf), hi(-inf, -inf, -inf);
  grow_aabb(*co->getCollisionShape(), solid.getCG2OTransform().inverse(), lo, hi);
  const Eigen::Vector3d half = (0.5 * eig(hi - lo)).cwiseMax(0.005);  // un sólido plano: 1 cm de grosor mínimo
  return core::RoundedBox{half, rounding_fraction * half.minCoeff(), eig(0.5 * (lo + hi))};
}

SolidContacts::SolidContacts(std::vector<Body> bodies)
: bodies_(std::move(bodies)),
  pose0_(bodies_.size(), Eigen::Isometry3d::Identity()),
  cg0_(bodies_.size(), Eigen::Vector3d::Zero()),
  force_(bodies_.size(), Eigen::Vector3d::Zero()),
  torque_(bodies_.size(), Eigen::Vector3d::Zero())
{
  snapshot_start();
}

void SolidContacts::snapshot_start()
{
  for (size_t b = 0; b < bodies_.size(); ++b) {
    pose0_[b] = isometry(o_now(bodies_[b].solid));
    cg0_[b] = eig(cg_now(bodies_[b].solid).getOrigin());
  }
}

std::vector<core::ContactBody> SolidContacts::contact_bodies() const
{
  std::vector<core::ContactBody> out;
  out.reserve(bodies_.size());
  for (size_t b = 0; b < bodies_.size(); ++b) {
    sf::SolidEntity* s = bodies_[b].solid;
    core::ContactBody c;
    c.box = bodies_[b].box;
    c.friction = static_cast<float>(bodies_[b].friction);
    c.pose0 = pose0_[b];
    c.pose1 = isometry(o_now(s));
    c.cg0 = cg0_[b];
    c.cg1 = eig(cg_now(s).getOrigin());
    c.v_cg = eig(s->getLinearVelocity());
    c.w = eig(s->getAngularVelocity());
    c.excluded_nodes = bodies_[b].excluded_nodes;
    out.push_back(std::move(c));
  }
  return out;
}

void SolidContacts::set_reactions(const std::vector<core::BodyReaction>& reactions, double dt, int substeps)
{
  for (size_t b = 0; b < bodies_.size() && b < reactions.size(); ++b) {
    force_[b] = core::reaction_force(reactions[b].momentum, dt, substeps);
    torque_[b] = core::reaction_force(reactions[b].angular, dt, substeps);
    // Nada que no sea finito llega a Bullet: un cuerpo con NaN lo saca de la simulación («Overflow in AABB»)
    if (!force_[b].allFinite() || !torque_[b].allFinite()) {
      force_[b].setZero();
      torque_[b].setZero();
    }
  }
}

void SolidContacts::add_reaction(size_t body, const Eigen::Vector3d& force, const Eigen::Vector3d& point)
{
  const Eigen::Vector3d torque = (point - eig(cg_now(bodies_[body].solid).getOrigin())).cross(force);
  if (!force.allFinite() || !torque.allFinite()) return;
  force_[body] += force;
  torque_[body] += torque;
}

Eigen::Isometry3d SolidContacts::pose_now(size_t body) const { return isometry(o_now(bodies_[body].solid)); }

void SolidContacts::apply_reactions()
{
  for (size_t b = 0; b < bodies_.size(); ++b) {
    if (!force_[b].isZero()) {
      bodies_[b].solid->ApplyCentralForce(bt(force_[b]));
      bodies_[b].solid->ApplyTorque(bt(torque_[b]));
    }
  }
}

std::vector<core::Wrench> SolidContacts::pending() const
{
  std::vector<core::Wrench> out;
  out.reserve(bodies_.size());
  for (size_t b = 0; b < bodies_.size(); ++b) out.push_back({force_[b], torque_[b]});
  return out;
}

}  // namespace stonefish_net
