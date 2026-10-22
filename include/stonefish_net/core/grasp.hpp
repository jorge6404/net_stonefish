// Agarre de la red con pinzas (docs/superpowers/specs/2026-10-20-agarre-pinza-design.md), sin ROS ni Stonefish.
//
// Una pinza son dos o más dedos, cada uno un cuerpo de contacto (su índice en la lista de cuerpos del solver, con su
// caja). Un nudo pellizcado, a menos de grip_margin de las cajas de dos dedos distintos de la misma pinza, se ata al
// primero de ellos (en el orden de los dedos) por el punto en que estaba en su marco. Se suelta cuando la distancia
// entre los centros de esos dos dedos crece más de release_gap desde que agarró: no hace falta saber nada de la
// pinza (ángulos, recorrido). También se prueba el punto medio de cada tramo con sus dos extremos libres (entonces
// se atan los dos): un hilo pellizcado entre dos nudos.
//
// Mientras está agarrado, el nudo (y sus tramos) no toca los dedos de su pinza: el otro dedo acaba atravesado por
// los tramos que salen del nudo. Al soltarlo sigue sin tocarlos hasta que ni él ni sus tramos solapan ninguno: si el
// contacto volviera de golpe, un dedo fino con tramos a los dos lados los empujaría siempre por la misma cara y
// lanzaría el nudo a través de la pinza (test_grasp). Hasta entonces, y mientras siga pellizcado, esa pinza no lo
// vuelve a agarrar: con release_gap < grip_margin soltaría y agarraría en cada tick.
//
// El atado es un core::Anchor con inv_mass = 0 (el dedo no cede dentro del tick): el tirón le llega en el tick
// siguiente, como el empuje del contacto.
#pragma once

#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "stonefish_net/core/contact.hpp"
#include "stonefish_net/core/net_topology.hpp"
#include "stonefish_net/core/xpbd.hpp"

namespace stonefish_net::core
{

// Un cuerpo de contacto visto como dedo
struct Finger
{
  Eigen::Isometry3d pose;  // marco del cuerpo → mundo
  RoundedBox box;          // en ese marco
};

struct Gripper
{
  double grip_margin, release_gap;  // m
  std::vector<size_t> fingers;      // índices de cuerpo de contacto, al menos dos
};

// Lo que hizo una pinza en un update: nudos que agarró, que soltó y los que tiene después
struct GraspEvent
{
  size_t gripper;
  int grabbed, released, held;
};

// Un nudo agarrado atado a su dedo durante un tick: el anclaje y el cuerpo que recibe su tirón
struct GripAnchor
{
  Anchor anchor;
  size_t body;
};

class Grasp
{
public:
  // anchored: nudos que ya tienen anclaje (el extremo de cada orinque) y no se agarran; tampoco los fijos
  Grasp(const NetTopology& topo, std::vector<Gripper> grippers, std::vector<int> anchored);

  // Pre-tick: x = nudos al final del tick anterior; bodies[b] = el cuerpo de contacto b al inicio del tick. Primero
  // suelta y luego agarra. Un dedo con pose no finita no agarra ni suelta. Solo devuelve las pinzas que han
  // agarrado o soltado algo.
  std::vector<GraspEvent> update(const std::vector<Eigen::Vector3f>& x, const std::vector<Finger>& bodies);

  // Post-tick: un anclaje por nudo agarrado, del punto atado con la pose de su dedo en update (from) al punto con
  // end[b], la pose del cuerpo al final del tick (to), con inv_mass = 0. Omite los de un dedo con pose no finita,
  // al inicio o al final.
  std::vector<GripAnchor> anchors(const std::vector<Eigen::Isometry3d>& end) const;

  // Añade a los dedos de cada pinza los nudos que no tocan: los que tiene agarrados y los soltados que aún los solapan
  void exclude_held(std::vector<ContactBody>& bodies) const;

  // Nudos que tiene agarrados la pinza
  std::vector<int> held(size_t gripper) const;

private:
  struct Hold
  {
    int node;
    size_t gripper, finger, partner;  // dedos en el orden de la pinza
    Eigen::Vector3d local;            // punto atado en el marco del dedo
    double gap;                       // distancia entre los centros de las cajas de finger y partner al agarrar
  };
  bool grabbable(int k) const { return !blocked_[static_cast<size_t>(k)] && owner_[static_cast<size_t>(k)] < 0; }
  // Si el nudo k y sus tramos están fuera de todos los dedos (con pose finita) y él no está a menos de margin de dos
  bool clear_of(int k, const std::vector<Eigen::Vector3f>& x, const std::vector<Finger>& fingers, double margin) const;
  // Nudos que los dedos de esa pinza no tocan
  std::vector<int> excluded(size_t gripper) const;

  const NetTopology& topo_;
  Adjacency adj_;
  std::vector<Gripper> grippers_;
  std::vector<bool> blocked_;  // fijos y anclados
  std::vector<int> owner_;     // por nudo, la pinza que lo tiene o −1
  std::vector<Hold> holds_;
  std::vector<std::pair<int, size_t>> releasing_;       // soltados (nudo, pinza) que aún solapan o pellizcan sus dedos
  std::vector<std::vector<Eigen::Isometry3d>> start_;  // por pinza, poses de sus dedos en el último update
};

}  // namespace stonefish_net::core
