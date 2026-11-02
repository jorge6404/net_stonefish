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

// Dos tramos del mismo color no comparten nudo y cada tramo tiene un solo color; las diagonales de los rombos
// (rigidez de ángulo) van en colores propios detrás de los de los tramos, que son los únicos que tocan los cuerpos
TEST(Topology, Colors)
{
  core::NetConfig cfg = config();
  cfg.net.knot_stiffness = 0.05;
  const core::NetTopology topo = core::build_topology(cfg);
  const core::LinkColoring c = core::color_links(topo);
  const size_t links = topo.links.size(), all = links + topo.shear.size();
  ASSERT_FALSE(topo.shear.empty());
  ASSERT_EQ(c.order.size(), all);
  ASSERT_FALSE(c.color_begin.empty());
  ASSERT_EQ(c.color_begin.front(), 0);
  ASSERT_EQ(static_cast<size_t>(c.color_begin.back()), all) << "los colores no cubren todos los tramos";
  ASSERT_TRUE(c.contact_colors > 0 && c.contact_colors < static_cast<int>(c.color_begin.size()) - 1);
  ASSERT_EQ(static_cast<size_t>(c.color_begin[static_cast<size_t>(c.contact_colors)]), links)
      << "los colores de contacto no son exactamente los tramos";
  const auto link_of = [&](int j) -> const core::Link& {
    return j < static_cast<int>(links) ? topo.links[static_cast<size_t>(j)] : topo.shear[static_cast<size_t>(j) - links];
  };
  std::vector<int> seen(all, 0);
  for (size_t col = 0; col + 1 < c.color_begin.size(); ++col) {
    std::set<int> nodes;
    for (int i = c.color_begin[col]; i < c.color_begin[col + 1]; ++i) {
      const int j = c.order[static_cast<size_t>(i)];
      ASSERT_EQ(static_cast<int>(col) < c.contact_colors, j < static_cast<int>(links)) << "tramo " << j << " en el color " << col;
      ++seen[static_cast<size_t>(j)];
      for (int n : {link_of(j).a, link_of(j).b}) {
        ASSERT_TRUE(nodes.insert(n).second) << "el color " << col << " repite el nudo " << n;
      }
    }
  }
  for (size_t j = 0; j < all; ++j) ASSERT_EQ(seen[j], 1) << "el tramo " << j;
  EXPECT_EQ(static_cast<int>(c.color_begin.size()) - 1 - c.contact_colors, 4)
      << "las diagonales (cadenas horizontales y verticales) caben en 4 colores: cada uno es un lanzamiento por subpaso";
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

// Las diagonales se empaquetan detrás de los tramos con w = 1 (empujan además de tirar) y sin diámetro (ni arrastre
// ni masa); los tramos, con w = 0. La adyacencia (arrastre) y el contacto solo ven los tramos.
TEST(Topology, GpuLayoutShear)
{
  core::NetConfig cfg = config();
  cfg.net.knot_stiffness = 0.05;
  const core::NetTopology topo = core::build_topology(cfg);
  const core::LinkColoring c = core::color_links(topo);
  const core::GpuLayout L = core::make_gpu_layout(topo, c);
  const size_t links = topo.links.size();
  ASSERT_EQ(L.link_count(), links + topo.shear.size());
  ASSERT_EQ(L.contact_colors, c.contact_colors);
  ASSERT_EQ(L.contact_link_count(), links);
  for (size_t i = 0; i < L.link_count(); ++i) {
    const int j = c.order[i];
    const bool shear = j >= static_cast<int>(links);
    const core::Link& l = shear ? topo.shear[static_cast<size_t>(j) - links] : topo.links[static_cast<size_t>(j)];
    ASSERT_TRUE(L.links[2 * i] == l.a && L.links[2 * i + 1] == l.b) << "el tramo empaquetado " << i;
    ASSERT_EQ(L.link_props[4 * i + 2], static_cast<float>(l.compliance));
    ASSERT_EQ(L.link_props[4 * i + 3], shear ? 1.f : 0.f) << "el tramo empaquetado " << i;
    if (shear) {
      ASSERT_EQ(L.link_props[4 * i + 1], 0.f);
    }
  }
  for (int i : L.adj_links) ASSERT_LT(static_cast<size_t>(i), links) << "la adyacencia apunta a una diagonal";
}

// Rigidez de ángulo en el nudo (net.knot_stiffness, N·m/rad): cada rombo de hilo completo tiene sus dos diagonales,
// con la longitud de reposo de la red sin deformar y flexibilidad d_otra² / (8 · k), la que da a cada esquina del
// rombo la rigidez k frente a un cambio de su ángulo. Sin rigidez (0), ninguna; un rombo roto, ninguna.
TEST(Topology, KnotStiffness)
{
  EXPECT_TRUE(topology().shear.empty()) << "sin knot_stiffness no hay diagonales";
  core::NetConfig cfg = config();
  cfg.net.knot_stiffness = 0.05;
  const core::NetTopology topo = core::build_topology(cfg);
  ASSERT_FALSE(topo.shear.empty());
  std::set<std::pair<int, int>> twine;
  for (const auto& l : topo.links) {
    if (l.kind == core::LinkKind::Twine) twine.insert(std::minmax(l.a, l.b));
  }
  const auto linked = [&](int a, int b) { return twine.count(std::minmax(a, b)) > 0; };
  const core::Adjacency adj = core::node_links(topo);
  const double diagonal = 2.0 * cfg.net.bar * std::sin(cfg.net.opening_deg * M_PI / 360.0);  // 90°: las dos iguales
  std::set<std::pair<int, int>> seen;
  for (const auto& d : topo.shear) {
    EXPECT_EQ(d.kind, core::LinkKind::Shear);
    EXPECT_TRUE(seen.insert(std::minmax(d.a, d.b)).second) << "diagonal repetida " << d.a << "-" << d.b;
    EXPECT_FALSE(linked(d.a, d.b));
    const double dist = (topo.nodes[static_cast<size_t>(d.a)].position - topo.nodes[static_cast<size_t>(d.b)].position).norm();
    EXPECT_NEAR(d.rest_length, dist, 1e-12);
    EXPECT_NEAR(d.rest_length, diagonal, 1e-9);
    EXPECT_NEAR(d.compliance, diagonal * diagonal / (8.0 * 0.05), 1e-9);
    // Sus extremos son esquinas opuestas de un rombo: dos nudos unidos por hilo a los dos
    int corners = 0;
    for (int a = adj.begin[static_cast<size_t>(d.a)]; a < adj.begin[static_cast<size_t>(d.a) + 1]; ++a) {
      const core::Link& l = topo.links[static_cast<size_t>(adj.links[static_cast<size_t>(a)])];
      const int m = l.a == d.a ? l.b : l.a;
      if (l.kind == core::LinkKind::Twine && linked(m, d.b)) ++corners;
    }
    EXPECT_EQ(corners, 2) << "la diagonal " << d.a << "-" << d.b << " no cruza un rombo completo";
  }
  // Un nudo del interior de la red está en cuatro rombos: dos diagonales horizontales y dos verticales
  std::vector<int> degree(topo.nodes.size(), 0);
  for (const auto& d : topo.shear) ++degree[static_cast<size_t>(d.a)], ++degree[static_cast<size_t>(d.b)];
  EXPECT_EQ(*std::max_element(degree.begin(), degree.end()), 4);
}

namespace
{
// Ángulo de reposo de cada rombo con diagonales: el de las esquinas de su diagonal más vertical (el opening_deg de
// la red regular), de las longitudes de reposo de sus dos diagonales; y su centro (y, z)
struct RhombusRest { double angle_deg, y, z; };
std::vector<RhombusRest> rhombus_rest(const core::NetTopology& topo)
{
  std::vector<RhombusRest> out;
  for (size_t i = 0; i + 1 < topo.shear.size(); i += 2) {  // add_shear las guarda de dos en dos, rombo a rombo
    const core::Link& p = topo.shear[i];
    const core::Link& q = topo.shear[i + 1];
    const auto dir = [&](const core::Link& l) {
      return (topo.nodes[static_cast<size_t>(l.b)].position - topo.nodes[static_cast<size_t>(l.a)].position).cwiseAbs();
    };
    const bool p_vertical = dir(p).z() > dir(p).y();
    const double dv = p_vertical ? p.rest_length : q.rest_length, dh = p_vertical ? q.rest_length : p.rest_length;
    const Eigen::Vector3d c = 0.5 * (topo.nodes[static_cast<size_t>(p.a)].position + topo.nodes[static_cast<size_t>(p.b)].position);
    out.push_back({2.0 * std::atan2(dh, dv) * 180.0 / M_PI, c.y(), c.z()});
  }
  return out;
}
}  // namespace

// Red irregular (irregular.angle_deg): el ángulo de reposo de cada rombo sigue un campo aleatorio suave alrededor de
// opening_deg, con esa desviación y correlación a la escala `scale` (rombos vecinos parecidos, lejanos
// independientes); la misma semilla da la misma red. Sin la sección, todos a opening_deg.
TEST(Topology, IrregularAngle)
{
  core::NetConfig cfg = config();
  cfg.net.knot_stiffness = 0.05;
  for (const auto& r : rhombus_rest(core::build_topology(cfg))) ASSERT_NEAR(r.angle_deg, cfg.net.opening_deg, 1e-6);
  cfg.irregular = {12.0, 0.0, 0.5, 3};
  const auto rest = rhombus_rest(core::build_topology(cfg));
  ASSERT_GT(rest.size(), 1000u);
  double sum = 0.0, sq = 0.0;
  for (const auto& r : rest) sum += r.angle_deg, sq += r.angle_deg * r.angle_deg;
  const double mean = sum / rest.size(), sd = std::sqrt(sq / rest.size() - mean * mean);
  std::printf("IrregularAngle: %zu rombos, media %.1f°, desviación %.1f°\n", rest.size(), mean, sd);
  EXPECT_NEAR(mean, cfg.net.opening_deg, 4.0);
  EXPECT_NEAR(sd, 12.0, 4.0);
  // Correlación entre pares de rombos cercanos (< scale/5) y lejanos (> 3·scale)
  double near_xy = 0, near_n = 0, far_xy = 0, far_n = 0;
  for (size_t i = 0; i < rest.size(); i += 7) {
    for (size_t j = i + 1; j < rest.size(); j += 13) {
      const double d = std::hypot(rest[i].y - rest[j].y, rest[i].z - rest[j].z);
      const double prod = (rest[i].angle_deg - mean) * (rest[j].angle_deg - mean) / (sd * sd);
      if (d < 0.1) near_xy += prod, ++near_n;
      if (d > 1.5) far_xy += prod, ++far_n;
    }
  }
  ASSERT_GT(near_n, 20);
  EXPECT_GT(near_xy / near_n, 0.7) << "rombos vecinos deberían tener ángulos parecidos";
  EXPECT_LT(std::abs(far_xy / far_n), 0.2) << "rombos lejanos deberían ser independientes";
  const auto again = rhombus_rest(core::build_topology(cfg));
  ASSERT_EQ(again.size(), rest.size());
  for (size_t i = 0; i < rest.size(); ++i) ASSERT_EQ(again[i].angle_deg, rest[i].angle_deg) << "misma semilla, otra red";
  cfg.irregular.seed = 4;
  const auto other = rhombus_rest(core::build_topology(cfg));
  EXPECT_NE(other[rest.size() / 2].angle_deg, rest[rest.size() / 2].angle_deg);
}

// irregular.length: la longitud natural de cada hilo cambia en (1 + length · campo suave) respecto a la distancia
// inicial (holguras y tirantes por zonas); los cabos no
TEST(Topology, IrregularLength)
{
  core::NetConfig cfg = config();
  cfg.irregular = {0.0, 0.03, 0.5, 5};
  const core::NetTopology plain = core::build_topology(config()), irr = core::build_topology(cfg);
  ASSERT_EQ(plain.links.size(), irr.links.size());
  double sum = 0.0, sq = 0.0;
  int n = 0;
  for (size_t j = 0; j < irr.links.size(); ++j) {
    const double ratio = irr.links[j].rest_length / plain.links[j].rest_length;
    if (irr.links[j].kind != core::LinkKind::Twine) {
      EXPECT_EQ(ratio, 1.0) << "un cabo cambió de longitud";
      continue;
    }
    EXPECT_GT(ratio, 0.85);
    sum += ratio, sq += ratio * ratio, ++n;
  }
  const double mean = sum / n, sd = std::sqrt(sq / n - mean * mean);
  std::printf("IrregularLength: %d hilos, media %.4f, desviación %.4f\n", n, mean, sd);
  EXPECT_NEAR(mean, 1.0, 0.01);
  EXPECT_NEAR(sd, 0.03, 0.01);
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

// Una línea pretensada (slack < 1) sale recta de su esquina a su pared, con la longitud natural de sus tramos
// slack × la distancia
TEST(Topology, TenseWallMooring)
{
  core::NetConfig cfg = config();
  cfg.mooring.bottom.to = core::NetConfig::LineTo::None;
  cfg.mooring.top = {core::NetConfig::LineTo::Wall, 0.0, 0.8};
  cfg.mooring.wall_y = 4.0;
  cfg.mooring.wall_z = 0.35;
  const core::NetTopology topo = core::build_topology(cfg);
  ASSERT_EQ(topo.marks.moorings.size(), 2u);
  double rest = 0.0, now = 0.0;
  for (const auto& l : topo.links) {
    if (l.kind != core::LinkKind::Mooring) continue;
    rest += l.rest_length;
    now += (topo.nodes[static_cast<size_t>(l.a)].position - topo.nodes[static_cast<size_t>(l.b)].position).norm();
  }
  double straight = 0.0;
  for (const auto& m : topo.marks.moorings) {
    const Eigen::Vector3d corner = topo.nodes[static_cast<size_t>(m.corner)].position;
    straight += std::abs((corner.y() > cfg.pose.y_center ? 3.99 : -3.99) - corner.y());
  }
  EXPECT_NEAR(now, straight, 1e-9) << "la línea pretensada no sale recta";
  EXPECT_NEAR(rest, 0.8 * straight, 1e-9);
}

}  // namespace stonefish_net::test

namespace stonefish_net::test
{

// Cada roto deja su centro y tamaño en coordenadas de la red (u desde el borde izquierdo visto
// desde −x, v desde la relinga hacia abajo), dentro de la red y de al menos un rombo
TEST(Topology, TearInfo)
{
  const core::NetConfig& cfg = config();
  const core::NetTopology& topo = topology();
  ASSERT_FALSE(topo.damage.tear_sizes.empty());
  ASSERT_EQ(topo.damage.tears.size(), topo.damage.tear_sizes.size());
  const double half = cfg.net.opening_deg * M_PI / 360.0;
  for (const auto& t : topo.damage.tears) {
    EXPECT_GT(t.u, 0.0);
    EXPECT_LT(t.u, cfg.net.width);
    EXPECT_GT(t.v, 0.0);
    EXPECT_LT(t.v, cfg.net.height);
    EXPECT_GE(t.w, 2.0 * cfg.net.bar * std::sin(half) - 1e-12);
    EXPECT_GE(t.h, 2.0 * cfg.net.bar * std::cos(half) - 1e-12);
  }
}

}  // namespace stonefish_net::test
