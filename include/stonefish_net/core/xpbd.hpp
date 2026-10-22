// Vocabulario del solver XPBD de la red, común a la GPU (GpuSolver, gpu/GpuXpbd), a la referencia en C++
// (core/xpbd_ref) y, lo de los cuerpos de contacto, a BulletSolver: anclajes, cuerpos que tocan la red, sus
// reacciones y las cuentas de acoplamiento con los sólidos de Bullet. Sin ROS ni Stonefish.
#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "stonefish_net/core/contact.hpp"

namespace stonefish_net::core
{

// Nudo unido al ojal de su boya. Sin la red, el ojal iría durante el tick de `from` (inicio) a `to` (final): en el
// subpaso i de n, a from + (i + 1)/n · (to − from), sin saltar todo el tick en el primer subpaso. La red lo
// aparta de ese camino según inv_mass, la inversa de la masa efectiva de la boya en el ojal (point_inv_mass): el
// nudo y el ojal se encuentran donde dicen sus masas, como en el ancla de Bullet. Con inv_mass = 0, el ojal no
// cede (boya de masa infinita). Al acabar el tick, quien mueve la boya la desplaza lo que la red apartó el ojal
// y le da en el ojal la velocidad de lo que este se movió en el tick (eye_impulse), como a un nudo en XPBD: así
// empieza el tick siguiente donde y como la dejó la red. (La velocidad que deja el impulso del ancla no sirve:
// Bullet ha aplicado a la boya todo su empuje, ~110 m/s² en una de 0,5 kg, antes de que la red tire, y el impulso
// lo compensa en la posición pero no en la velocidad del final.)
//
// Sin esto (boya que no cede y recibe la reacción en el tick siguiente) el acoplamiento es explícito y diverge si
// lo que cuelga pesa más que la boya: test_xpbd, Xpbd.BuoysSettle.
struct Anchor
{
  int node;
  Eigen::Vector3f from, to;  // m, mundo
  Eigen::Matrix3f inv_mass = Eigen::Matrix3f::Zero();  // 1/kg
};

struct XpbdParams
{
  float h;           // s, subpaso
  float rho, g;      // densidad del agua, gravedad
  float cn, ct;      // arrastre normal y tangencial
  float compliance;  // m/N
  float damping;     // por subpaso (substep_damping)
  float friction;    // μ con el suelo
  float floor_z;     // m (NED: más hondo = z mayor)
};

// Un sólido que toca la red durante un tick. En el subpaso i de n su pose es la interpolación entre pose0 (inicio
// del tick) y pose1 (final) en (i + 1)/n; la velocidad de su superficie sale de v_cg y w (las del final del tick).
struct ContactBody
{
  RoundedBox box;
  float friction;
  Eigen::Isometry3d pose0, pose1;  // marco del cuerpo → mundo
  Eigen::Vector3d cg0, cg1;        // centro de gravedad al inicio y al final, mundo
  Eigen::Vector3d v_cg, w;         // velocidad lineal del centro de gravedad y angular
  std::vector<int> excluded_nodes;  // no tocan este cuerpo (el orinque de su propia boya, lo agarrado por un dedo)
};

// Lo que la red movió por un cuerpo en un tick: Σ m·Δx y Σ r × m·Δx respecto a su centro de gravedad. El cuerpo
// recibe reaction_force(momentum) y reaction_force(angular) en el pre-tick siguiente.
struct BodyReaction
{
  Eigen::Vector3d momentum, angular;
};

// Por punto (nudo o tramo), cuántos cuerpos recuerda el contacto (GPU y referencia)
inline constexpr int kMaxContactBodies = 4;

// Por nudo, los cuerpos que no toca: bit b = el nudo está en bodies[b].excluded_nodes. Solo caben los
// kMaxExcludingBodies primeros cuerpos; los demás no pueden excluir nada (no ponen bit). Body: cualquier tipo con
// excluded_nodes (ContactBody, SolidContacts::Body).
inline constexpr size_t kMaxExcludingBodies = 32;

template <typename Body>
std::vector<uint32_t> exclusion_masks(const std::vector<Body>& bodies, size_t nodes)
{
  std::vector<uint32_t> masks(nodes, 0u);
  for (size_t b = 0; b < bodies.size() && b < kMaxExcludingBodies; ++b) {
    for (int k : bodies[b].excluded_nodes) masks[static_cast<size_t>(k)] |= 1u << b;
  }
  return masks;
}

// Si la máscara de un nudo dice que no toca el cuerpo b
inline bool excludes(uint32_t mask, size_t b) { return b < kMaxExcludingBodies && (mask >> b & 1u) != 0u; }

// Los cuerpos que excluyen nudos y no caben en la máscara (índice >= kMaxExcludingBodies): chocan con ellos
template <typename Body>
std::vector<size_t> cannot_exclude(const std::vector<Body>& bodies)
{
  std::vector<size_t> out;
  for (size_t b = kMaxExcludingBodies; b < bodies.size(); ++b) {
    if (!bodies[b].excluded_nodes.empty()) out.push_back(b);
  }
  return out;
}

// Traslación lineal y giro esférico (slerp) entre a (s = 0) y b (s = 1)
Eigen::Isometry3d interpolate_pose(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b, double s);

// Fuerza sobre un cuerpo (o boya) por lo que la red movió por él en un tick de n subpasos: −n·Σ m·Δx/dt².
// Cada subpaso vuelve a integrar la velocidad, así que una corrección Δx cambia la cantidad de movimiento en m·Δx/h
// (no m·Δx/dt, como con la única pasada por tick de Bullet, n = 1): el impulso del tick es Σ m·Δx/h y la fuerza
// media, Σ m·Δx/(h·dt). Comprobado en test_xpbd: un ancla recibe el peso de lo que cuelga de ella.
Eigen::Vector3d reaction_force(const Eigen::Vector3d& moved, double dt, int substeps);

// Impulso en el punto de un sólido con esa inversa de masa efectiva (point_inv_mass) que cambia su velocidad ahí
// en dv: inv_mass⁻¹ · dv.
Eigen::Vector3d eye_impulse(const Eigen::Matrix3d& inv_mass, const Eigen::Vector3d& dv);

// Inversa de la masa efectiva de un sólido en un punto a r de su centro de gravedad (mundo): un impulso f en el
// punto le cambia la velocidad ahí en inv_mass · f = f/m + (I⁻¹ (r × f)) × r. inv_inertia: I⁻¹ en el mundo.
Eigen::Matrix3f point_inv_mass(double mass, const Eigen::Matrix3d& inv_inertia, const Eigen::Vector3d& r);

// Amortiguamiento por subpaso que en n subpasos quita lo mismo que `damping` en un tick: 1 − (1 − damping)^(1/n).
float substep_damping(double damping, int substeps);

}  // namespace stonefish_net::core
