// Pose de un sólido dentro de un tick de Bullet (en los callbacks pre-tick y post-tick).
//
// SolidEntity::getCGTransform de un sólido libre lee el estado de movimiento de su btRigidBody, y Bullet solo lo
// escribe después de cada tick (synchronizeMotionStates, tras el post-tick) y además interpolado hacia atrás
// (m_latencyMotionStateInterpolation): en el pre-tick y en el post-tick de un mismo tick vale lo mismo y va por
// detrás de la pose real una fracción de tick que depende del reloj. Para la red el sólido parecería quieto todo el
// tick y saltaría al empezar el siguiente. Aquí se lee la del btRigidBody. Los enlaces de robot (multibody) no
// tienen ese retraso: su getCGTransform ya es la del collider.
#pragma once

#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <Stonefish/entities/SolidEntity.h>

namespace stonefish_net
{

inline btTransform cg_now(sf::SolidEntity* s)
{
  if (btRigidBody* rb = s->getRigidBody()) return rb->getCenterOfMassTransform();
  return s->getCGTransform();
}

inline btTransform o_now(sf::SolidEntity* s) { return cg_now(s) * s->getCG2OTransform(); }

}  // namespace stonefish_net
