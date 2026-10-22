// Topología de la red: un grafo de nudos y tramos más los puntos de referencia que apuntan a él
// (boyas, plomos, fondeo). La construcción está en etapas (src/core/topology/): retícula, aparejo,
// daño, recorte a la pieza principal y masas.
#pragma once

#include <vector>

#include <Eigen/Core>

#include "stonefish_net/core/net_config.hpp"

namespace stonefish_net::core
{

enum class LinkKind { Twine, Frame, Dropper, Mooring };
enum class NodeKind { Knot, Dropper, Mooring };  // nudo de la red (incluido el marco), orinque o fondeo

struct Node
{
  Eigen::Vector3d position;
  NodeKind kind;
  double mass = 0;           // kg reales (peso)
  double inertial_mass = 0;  // mass + masa añadida del agua
  double volume = 0;         // m³ (empuje)
  bool fixed = false;        // extremo del fondeo: masa 0 en Bullet
};

struct Link
{
  int a, b;
  // Siempre la distancia inicial entre a y b: Bullet toma de ahí la longitud natural (appendLink y
  // btSoftBody::updateConstants la recalculan), así que la holgura de un tramo tiene que estar en la forma.
  double rest_length;
  double diameter;
  LinkKind kind;
};

struct Strand
{
  std::vector<int> nodes;  // ordenados, para el render
  LinkKind kind;
};

struct BuoyMount
{
  int node;             // extremo superior del orinque
  Eigen::Vector3d eye;  // posición del ojal de la boya
};

struct MooringLine
{
  int corner;                  // esquina de la red de la que sale
  Eigen::Vector3d block_base;  // base del bloque de anclaje, en el suelo
};

// Todo índice de nudo que no es parte del grafo vive aquí: al recortar o renumerar el grafo
// basta con remap() y ningún índice queda apuntando a un nudo equivocado.
struct Landmarks
{
  std::vector<BuoyMount> buoys;
  std::vector<int> leads;
  std::vector<MooringLine> moorings;  // inferior izq., inferior der., superior izq., superior der.

  void remap(const std::vector<int>& old_to_new);
};

struct DamageReport
{
  std::vector<int> tear_sizes;  // rombos de cada roto
  int loose_cut = 0;            // hilos sueltos cortados
};

struct NetTopology
{
  std::vector<Node> nodes;
  std::vector<Link> links;
  std::vector<Strand> strands;
  Landmarks marks;
  DamageReport damage;
};

// Lanza std::invalid_argument si la geometría no es construible (p. ej. "mooring.floor_z: …").
NetTopology build_topology(const NetConfig& cfg);

// Lista dispersa (CSR) nudo → tramos que lo tocan: los del nudo k son links[begin[k], begin[k + 1]).
struct Adjacency
{
  std::vector<int> begin;  // nudos + 1
  std::vector<int> links;
};

Adjacency node_links(const NetTopology& topo);

// Los nudos del orinque que sale de `mount` (su extremo superior, el que va al ojal de la boya), con él incluido
std::vector<int> dropper_nodes(const NetTopology& topo, int mount);

}  // namespace stonefish_net::core
