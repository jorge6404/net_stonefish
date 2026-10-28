// Topología de la red: colores de tramos, adyacencia, empaquetado para la GPU y orinques
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
#include <vector>

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

// La flexibilidad de cada tramo es L / (E · π d²/4) con el módulo del hilo o el del cabo; con .inf (la red de
// prueba), 0
TEST(Topology, LinkCompliance)
{
  for (const auto& l : topology().links) EXPECT_EQ(l.compliance, 0.0) << "con módulo .inf un tramo no es inextensible";
  core::NetConfig cfg = config();
  cfg.net.twine_modulus = 1e9;
  cfg.frame_rope.modulus = 0.5e9;
  const core::NetTopology topo = core::build_topology(cfg);
  bool twine = false, rope = false;
  for (const auto& l : topo.links) {
    const double modulus = l.kind == core::LinkKind::Twine ? 1e9 : 0.5e9;
    EXPECT_NEAR(l.compliance, l.rest_length / (modulus * M_PI * l.diameter * l.diameter / 4.0), 1e-12 * l.compliance);
    (l.kind == core::LinkKind::Twine ? twine : rope) = true;
  }
  EXPECT_TRUE(twine && rope);
}

// Con knot_diameter, cada nudo de hilo gana la esfera de su nudo (masa y volumen, con la densidad del hilo) y su
// diámetro; los de orinque no
TEST(Topology, KnotMass)
{
  core::NetConfig cfg = config();
  cfg.net.knot_diameter = 0.006;
  const core::NetTopology plain = core::build_topology(config()), knotted = core::build_topology(cfg);
  ASSERT_EQ(plain.nodes.size(), knotted.nodes.size());
  const double volume = M_PI * std::pow(0.006, 3) / 6.0;
  const core::Adjacency adj = core::node_links(plain);
  int knots = 0, droppers = 0;
  for (size_t k = 0; k < plain.nodes.size(); ++k) {
    bool has_twine = false;
    for (int a = adj.begin[k]; a < adj.begin[k + 1]; ++a) {
      has_twine |= plain.links[static_cast<size_t>(adj.links[static_cast<size_t>(a)])].kind == core::LinkKind::Twine;
    }
    const double dm = knotted.nodes[k].mass - plain.nodes[k].mass, dv = knotted.nodes[k].volume - plain.nodes[k].volume;
    if (plain.nodes[k].kind == core::NodeKind::Knot && has_twine) {
      EXPECT_NEAR(dm, volume * cfg.net.twine_density, 1e-12);
      EXPECT_NEAR(dv, volume, 1e-15);
      EXPECT_EQ(knotted.nodes[k].knot_diameter, 0.006);
      ++knots;
    } else if (plain.nodes[k].kind == core::NodeKind::Dropper) {
      EXPECT_EQ(dm, 0.0);
      EXPECT_EQ(knotted.nodes[k].knot_diameter, 0.0);
      ++droppers;
    }
  }
  EXPECT_GT(knots, 0);
  EXPECT_GT(droppers, 0);
}

// Sin boyas: ni boyas ni nudos ni tramos de orinque, y la misma red
TEST(Topology, NoBuoys)
{
  core::NetConfig cfg = config();
  cfg.buoys.enabled = false;
  const core::NetTopology topo = core::build_topology(cfg);
  EXPECT_TRUE(topo.marks.buoys.empty());
  for (const auto& n : topo.nodes) EXPECT_NE(n.kind, core::NodeKind::Dropper);
  for (const auto& l : topo.links) EXPECT_NE(l.kind, core::LinkKind::Dropper);
  const auto knots = [](const core::NetTopology& t) {
    return std::count_if(t.nodes.begin(), t.nodes.end(), [](const core::Node& n) { return n.kind == core::NodeKind::Knot; });
  };
  EXPECT_EQ(knots(topo), knots(topology()));
}

// Colgada de las paredes: solo las esquinas superiores, cada una con su línea hasta un nudo fijo a 1 cm de su pared
// (frente a ella, a la altura wall_z) y sin bloque; una pared que no queda por fuera de la red se rechaza
TEST(Topology, WallMooring)
{
  core::NetConfig cfg = config();
  cfg.mooring.bottom.to = core::NetConfig::LineTo::None;
  cfg.mooring.top = {core::NetConfig::LineTo::Wall, 0.0, 1.0};
  cfg.mooring.wall_y = 4.0;
  cfg.mooring.wall_z = 0.35;
  const core::NetTopology topo = core::build_topology(cfg);
  ASSERT_EQ(topo.marks.moorings.size(), 2u);
  std::vector<Eigen::Vector3d> fixed;
  for (const auto& n : topo.nodes) {
    if (n.fixed && n.kind == core::NodeKind::Mooring) fixed.push_back(n.position);
  }
  ASSERT_EQ(fixed.size(), 2u);
  for (size_t k = 0; k < 2; ++k) {
    const auto& m = topo.marks.moorings[k];
    EXPECT_FALSE(m.on_floor);
    const Eigen::Vector3d corner = topo.nodes[static_cast<size_t>(m.corner)].position;
    const double side = corner.y() > cfg.pose.y_center ? 1.0 : -1.0;
    const Eigen::Vector3d tie(corner.x(), side * 3.99, 0.35);
    EXPECT_TRUE((fixed[0] - tie).norm() < 1e-9 || (fixed[1] - tie).norm() < 1e-9)
        << "la línea de la esquina y=" << corner.y() << " no acaba en su pared";
  }
  cfg.mooring.wall_y = 1.0;
  EXPECT_THROW(core::build_topology(cfg), std::invalid_argument);
}

}  // namespace stonefish_net::test
