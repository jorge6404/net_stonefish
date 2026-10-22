// Los cuerpos que tocan la red con contacto propio y no con el de Bullet (que solo mira los nudos y deja pasar un
// cuerpo entre ellos): enlaces de robot (robot_contact, robot_contact_auto) y sólidos dinámicos que no son robots
// (solid_contact_auto: boyas, objetos sueltos). Cada uno es una caja redondeada en el marco O de su sólido.
//
// La lista la usan los dos solvers: BulletSolver con RobotContact (en la CPU) y GpuSolver con contact_bodies. Los dos
// dejan aquí lo que la red movió por cada cuerpo (set_reactions) y este se lo devuelve como core::reaction_force (y
// su par) en el pre-tick siguiente (apply_reactions).
#pragma once

#include <optional>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "stonefish_net/core/contact.hpp"
#include "stonefish_net/core/xpbd.hpp"

namespace sf
{
class Robot;
class SolidEntity;
}  // namespace sf

namespace stonefish_net
{

class SolidContacts
{
public:
  struct Body
  {
    sf::Robot* robot;     // nullptr si no es un enlace de robot
    sf::SolidEntity* solid;
    core::RoundedBox box;  // en el marco O del sólido
    double friction;
    std::vector<int> excluded_nodes;  // no tocan este cuerpo (el orinque de su propia boya)
  };

  explicit SolidContacts(std::vector<Body> bodies = {});

  // Caja envolvente de la forma de colisión del sólido en su marco O, con radio de redondeo
  // rounding_fraction × su semieje menor; nada si no tiene forma de colisión.
  static std::optional<core::RoundedBox> fit_box(sf::SolidEntity& solid, double rounding_fraction);

  const std::vector<Body>& bodies() const { return bodies_; }

  // Pre-tick: pose y centro de gravedad de cada cuerpo al inicio del tick.
  void snapshot_start();

  // Post-tick, con los cuerpos ya movidos por Bullet: cada uno con su pose al inicio y al final del tick.
  std::vector<core::ContactBody> contact_bodies() const;

  // Lo que la red movió por cada cuerpo en un tick de n subpasos (en el orden de bodies()): queda pendiente
  // core::reaction_force.
  void set_reactions(const std::vector<core::BodyReaction>& reactions, double dt, int substeps);

  // Suma a la reacción pendiente del cuerpo una fuerza en un punto del mundo, ambos al final del tick (el tirón de
  // un nudo agarrado sobre su dedo): la fuerza y su par respecto al centro de gravedad de ahora. Después de
  // set_reactions, que la sobrescribe; lo no finito se descarta.
  void add_reaction(size_t body, const Eigen::Vector3d& force, const Eigen::Vector3d& point);

  // Pre-tick: aplica a cada cuerpo la reacción pendiente.
  void apply_reactions();

  // Marco O del cuerpo al inicio del tick (snapshot_start) y ahora
  const Eigen::Isometry3d& pose_start(size_t body) const { return pose0_[body]; }
  Eigen::Isometry3d pose_now(size_t body) const;

private:
  std::vector<Body> bodies_;
  std::vector<Eigen::Isometry3d> pose0_;
  std::vector<Eigen::Vector3d> cg0_, force_, torque_;
};

}  // namespace stonefish_net
