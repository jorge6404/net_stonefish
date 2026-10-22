// Contacto de la red con los cuerpos de SolidContacts (enlaces de robot y sólidos sueltos) cuando la red es el cuerpo
// blando de Bullet (BulletSolver). Sustituye al de Bullet, que con SDF_RS solo mira los nudos: el robot pasaría entre
// ellos. Cada cuerpo es una caja redondeada.
//
// Es una restricción de posición más dentro del bucle de posiciones del cuerpo blando (BulletSolver::end_step):
// en cada iteración, después de los tramos, cada nudo que ha entrado en la caja, y el punto más hundido de cada
// tramo que la cruza (aunque ninguno de sus extremos esté dentro: así una pieza más fina que la malla, como el
// dedo de una pinza, tampoco atraviesa los hilos), vuelve a la superficie con rozamiento de Coulomb. Al ir la
// última, la iteración acaba siempre con la red fuera de la caja aunque los tramos tiren de ella hacia dentro.
// Lo que corrige incluye esa tensión: la red devuelve al cuerpo la reacción −Σ m·Δx/dt², con su par respecto al
// centro de gravedad, que SolidContacts aplica en el pre-tick siguiente (las fuerzas del post-tick las borraría
// Stonefish).
#pragma once

#include <cstdint>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "stonefish_net/SolidContacts.h"
#include "stonefish_net/core/contact.hpp"
#include "stonefish_net/core/net_topology.hpp"

class btSoftBody;

namespace stonefish_net
{

class RobotContact
{
public:
  // La reacción de cada tick queda pendiente en `solids`, que tiene que vivir más que este objeto.
  RobotContact(SolidContacts& solids, const core::NetTopology& topo);

  // Por robot, con la pose actual, la esfera fuera de la cual ningún nudo puede tocar sus enlaces en un subpaso
  // (la del descarte grueso de begin_step).
  struct Sphere
  {
    Eigen::Vector3d center;
    double radius;
  };
  std::vector<Sphere> robot_spheres() const;

  // Un tick del bucle de posiciones: begin_step una vez (pose de cada cuerpo y nudos y tramos cercanos entre
  // `nodes`, los que se mueven), project en cada iteración, end_step al acabar (reacción pendiente en SolidContacts
  // y caras de entrada recordadas).
  void begin_step(const btSoftBody& body, double dt, const std::vector<int>& nodes);
  void project(btSoftBody& body);
  void end_step();

private:
  // Estado de un cuerpo durante un tick
  struct Frame
  {
    Eigen::Isometry3d pose;
    Eigen::Vector3d center;                // centro de la caja, marco mundo
    double reach;                          // radio de la esfera de candidatos alrededor de center
    Eigen::Vector3d cg, v_cg, w;
    std::vector<int> nodes, segments;     // candidatos: nudos libres y tramos cerca de la caja
    Eigen::Vector3d momentum, angular;    // Σ m·Δx y Σ r × m·Δx de la red en este subpaso
    std::unordered_map<int, Eigen::Vector3d> touching;  // en contacto en este subpaso → dirección de salida
  };

  void contact(size_t ti, int id, int a, int b, double s, btSoftBody& body);
  // Centro de la caja del cuerpo en el mundo y radio de la esfera de candidatos a su alrededor
  Sphere reach(const SolidContacts::Body& t, const Eigen::Isometry3d& pose) const;
  // Esfera que envuelve las de los enlaces de un robot
  static Sphere enclosing(const std::vector<Sphere>& spheres);

  SolidContacts& solids_;
  std::vector<std::vector<size_t>> groups_;    // índices de cuerpo por robot (un sólido suelto, solo)
  std::vector<std::pair<int, int>> segments_;  // tramos de la red
  std::vector<std::vector<int>> starting_;     // por nudo: tramos que empiezan en él (segments_[j].first)
  std::vector<uint32_t> excluded_;             // por nudo, los cuerpos que no toca (core::exclusion_masks)
  std::vector<int> group_nodes_;               // nudos dentro de la esfera del robot que se está tratando
  std::vector<double> group_d2_;               // su distancia² al centro de esa esfera
  std::vector<Sphere> group_spheres_;          // esferas de candidatos de los enlaces de ese robot
  double max_segment_ = 0;                     // longitud natural del tramo más largo, m
  int node_count_ = 0;
  double dt_ = 0;                              // subpaso en curso, s
  std::vector<Frame> frames_;
  // Por caja: dirección de salida (marco de la caja) de cada punto que estaba en contacto en el subpaso anterior,
  // la de la cara por la que entró. Clave: índice del nudo, o número de nudos + índice del tramo. Se olvida en
  // cuanto sale.
  std::vector<std::unordered_map<int, Eigen::Vector3d>> exits_;
};

}  // namespace stonefish_net
