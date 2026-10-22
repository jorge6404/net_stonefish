// Subpaso XPBD de la red en C++ normal (float, como la GPU): el oráculo con el que tools/gpu_check compara los
// shaders y con el que test_xpbd y test_grasp prueban el algoritmo. Mismo algoritmo y mismo orden que la GPU (spec:
// «Un tick en la GPU»):
//   a. fuerzas (peso − empuje y Morison de sus tramos, limitado) y predicción, con el estado del subpaso anterior
//   b. tramos por color: restricción de distancia XPBD solo a tracción
//   c. anclajes: nudo y ojal de su boya llevados al mismo punto, cada uno según su masa (core::Anchor)
//   c'. contacto con cajas redondeadas (sólidos): primero los nudos, luego los tramos por color
//   d. suelo z <= floor_z con rozamiento de Coulomb
//   e. velocidades a partir del desplazamiento del subpaso
//
// En float no se puede sacar la velocidad como (x − x_anterior)/h: con la red a unos metros del origen y
// desplazamientos de micras por subpaso, el redondeo de las posiciones dividido por h es ruido de décimas de m/s
// (crece con los subpasos) y la red nunca se para. Por eso el estado lleva aparte dx, el desplazamiento del
// subpaso: empieza en h·v y cada corrección se suma a la vez a x y a dx; la velocidad es dx/h.
#pragma once

#include <functional>
#include <vector>

#include <Eigen/Core>

#include "stonefish_net/core/gpu_layout.hpp"
#include "stonefish_net/core/xpbd.hpp"

namespace stonefish_net::core
{

struct XpbdState
{
  std::vector<Eigen::Vector3f> x, v, dx;  // posición, velocidad y desplazamiento del último subpaso
  // Por anclaje, dentro del tick: cuánto se ha apartado el ojal de su camino sin la red, y a qué velocidad
  std::vector<Eigen::Vector3f> anchor_offset, anchor_vel;
};

// Por punto (nudo o tramo), los cuerpos con los que estaba en contacto en el subpaso anterior y la dirección de
// salida (la de la cara por la que entró, en el marco de la caja): un punto que sigue dentro sale por ahí aunque
// haya pasado del centro de una pieza fina. xyz = dirección, w = índice del cuerpo (−1 = libre).
struct ContactMemory
{
  std::vector<Eigen::Vector4f> node, link;  // kMaxContactBodies por nudo / por tramo del layout
};

ContactMemory contact_memory(const GpuLayout& L);  // todo libre

XpbdState xpbd_init(const GpuLayout& L);

// Subpaso i de n. water(x): velocidad del agua en x. anchor_dx[k] += m·Δx del nudo del anclaje k y
// reactions[b] += lo del cuerpo b (para las reacciones); se redimensionan si hace falta y no se ponen a cero (lo
// hace quien lee, una vez por tick).
void xpbd_substep(const GpuLayout& L, const XpbdParams& p, const std::function<Eigen::Vector3f(const Eigen::Vector3f&)>& water,
                  const std::vector<Anchor>& anchors, const std::vector<ContactBody>& bodies, int i, int n, XpbdState& s,
                  ContactMemory& memory, std::vector<Eigen::Vector3f>& anchor_dx, std::vector<BodyReaction>& reactions);

}  // namespace stonefish_net::core
