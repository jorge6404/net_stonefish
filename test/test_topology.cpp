// Topología de la red: colores de tramos, adyacencia, empaquetado para la GPU y orinques
#include <gtest/gtest.h>

#include <set>

#include "support/scene.hpp"

namespace stonefish_net::test
{

// Dos tramos del mismo color no comparten nudo y cada tramo tiene un solo color
TEST(Topology, Colors)
{
  const core::NetTopology& topo = topology();
  const core::LinkColoring c = core::color_links(topo);
  const size_t links = topo.links.size();
  ASSERT_EQ(c.order.size(), links);
  ASSERT_FALSE(c.color_begin.empty());
  ASSERT_EQ(c.color_begin.front(), 0);
  ASSERT_EQ(static_cast<size_t>(c.color_begin.back()), links) << "los colores no cubren todos los tramos";
  std::vector<int> seen(links, 0);
  for (size_t col = 0; col + 1 < c.color_begin.size(); ++col) {
    std::set<int> nodes;
    for (int i = c.color_begin[col]; i < c.color_begin[col + 1]; ++i) {
      const int j = c.order[static_cast<size_t>(i)];
      ++seen[static_cast<size_t>(j)];
      for (int n : {topo.links[static_cast<size_t>(j)].a, topo.links[static_cast<size_t>(j)].b}) {
        ASSERT_TRUE(nodes.insert(n).second) << "el color " << col << " repite el nudo " << n;
      }
    }
  }
  for (size_t j = 0; j < links; ++j) ASSERT_EQ(seen[j], 1) << "el tramo " << j;
}

// La adyacencia nudo → tramos tiene exactamente los tramos de cada nudo
TEST(Topology, Adjacency)
{
  const core::NetTopology& topo = topology();
  const core::Adjacency adj = core::node_links(topo);
  ASSERT_EQ(adj.begin.size(), topo.nodes.size() + 1);
  std::vector<std::multiset<int>> expected(topo.nodes.size());
  for (size_t j = 0; j < topo.links.size(); ++j) {
    expected[static_cast<size_t>(topo.links[j].a)].insert(static_cast<int>(j));
    expected[static_cast<size_t>(topo.links[j].b)].insert(static_cast<int>(j));
  }
  for (size_t k = 0; k < topo.nodes.size(); ++k) {
    const std::multiset<int> got(adj.links.begin() + adj.begin[k], adj.links.begin() + adj.begin[k + 1]);
    ASSERT_EQ(got, expected[k]) << "la adyacencia del nudo " << k << " no coincide con sus tramos";
  }
}

// El empaquetado para la GPU sigue el orden de los colores (el tramo i del layout es order[i]) y su adyacencia usa
// índices de tramo del layout
TEST(Topology, GpuLayout)
{
  const core::NetTopology& topo = topology();
  const core::LinkColoring c = core::color_links(topo);
  const core::GpuLayout L = core::make_gpu_layout(topo, c);
  const size_t links = topo.links.size();
  ASSERT_EQ(L.x0.size(), 4 * topo.nodes.size());
  ASSERT_EQ(L.links.size(), 2 * links);
  ASSERT_EQ(L.link_props.size(), 4 * links);
  ASSERT_EQ(L.color_begin, c.color_begin);
  for (size_t i = 0; i < links; ++i) {
    const auto& l = topo.links[static_cast<size_t>(c.order[i])];
    ASSERT_TRUE(L.links[2 * i] == l.a && L.links[2 * i + 1] == l.b && L.link_props[4 * i] == static_cast<float>(l.rest_length))
        << "el tramo empaquetado " << i << " no es order[" << i << "]";
  }
  for (size_t k = 0; k < topo.nodes.size(); ++k) {
    const float w = topo.nodes[k].fixed ? 0.f : static_cast<float>(1.0 / topo.nodes[k].inertial_mass);
    ASSERT_EQ(L.x0[4 * k + 3], w) << "masa inversa del nudo " << k;
    for (int a = L.adj_begin[k]; a < L.adj_begin[k + 1]; ++a) {
      const auto i = static_cast<size_t>(L.adj_links[static_cast<size_t>(a)]);
      ASSERT_TRUE(L.links[2 * i] == static_cast<int>(k) || L.links[2 * i + 1] == static_cast<int>(k))
          << "la adyacencia empaquetada del nudo " << k << " apunta a un tramo ajeno";
    }
  }
}

// El orinque de cada boya: su extremo superior y nudos de orinque unidos por tramos de orinque, distintos para cada
// boya
TEST(Topology, DropperNodes)
{
  const core::NetTopology& topo = topology();
  ASSERT_FALSE(topo.marks.buoys.empty());
  std::set<int> all;
  for (const auto& b : topo.marks.buoys) {
    const std::vector<int> nodes = core::dropper_nodes(topo, b.node);
    ASSERT_FALSE(nodes.empty());
    EXPECT_EQ(nodes.front(), b.node);
    EXPECT_GT(nodes.size(), 1u) << "el orinque de la boya del nudo " << b.node << " no tiene más nudos";
    for (int k : nodes) {
      EXPECT_EQ(topo.nodes[static_cast<size_t>(k)].kind, core::NodeKind::Dropper);
      EXPECT_TRUE(all.insert(k).second) << "el nudo " << k << " está en dos orinques";
    }
  }
}

}  // namespace stonefish_net::test
