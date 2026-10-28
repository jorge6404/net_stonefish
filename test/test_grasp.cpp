// El agarre de la red con pinzas (core::Grasp): sus reglas sin solver y una pinza que agarra la red, tira y la
// suelta con el XPBD de referencia.
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdio>
#include <limits>
#include <tuple>

#include "support/scene.hpp"

namespace stonefish_net::test
{
namespace
{
bool holds(const core::Grasp& grasp, int node, size_t gripper)
{
  const std::vector<int> held = grasp.held(gripper);
  return std::find(held.begin(), held.end(), node) != held.end();
}

bool anchors_finite(const std::vector<core::GripAnchor>& anchors)
{
  return std::all_of(anchors.begin(), anchors.end(),
                     [](const core::GripAnchor& a) { return a.anchor.from.allFinite() && a.anchor.to.allFinite(); });
}

std::vector<Eigen::Isometry3d> poses(const std::vector<core::Finger>& fingers)
{
  std::vector<Eigen::Isometry3d> out;
  for (const auto& f : fingers) out.push_back(f.pose);
  return out;
}

// Dos pinzas sobre los mismos dedos: los cuerpos 0-1 son de la primera y los 2-3 de la segunda
std::vector<core::Finger> both(const std::vector<core::Finger>& first, const std::vector<core::Finger>& second)
{
  std::vector<core::Finger> out = first;
  out.insert(out.end(), second.begin(), second.end());
  return out;
}

// La red en su forma inicial y el nudo libre más cercano a su centro
struct Rest
{
  std::vector<Eigen::Vector3f> x;
  int k = -1;
  Eigen::Vector3d c;
};
Rest rest(const core::NetTopology& topo)
{
  Rest r;
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto& node : topo.nodes) {
    r.x.push_back(node.position.cast<float>());
    mean += node.position;
  }
  mean /= static_cast<double>(topo.nodes.size());
  for (size_t i = 0; i < topo.nodes.size(); ++i) {
    const auto& node = topo.nodes[i];
    if (node.kind == core::NodeKind::Knot && !node.fixed &&
        (r.k < 0 || (node.position - mean).norm() < (topo.nodes[static_cast<size_t>(r.k)].position - mean).norm())) {
      r.k = static_cast<int>(i);
    }
  }
  r.c = topo.nodes[static_cast<size_t>(r.k)].position;
  return r;
}
}  // namespace

// Dos dedos a ±6 mm en x de un nudo libre lo agarran, y con los dedos quietos el anclaje está en el nudo; abrir
// menos que release_gap no suelta
TEST(Grasp, PinchGrabsAndHolds)
{
  const Rest r = rest(topology());
  const auto closed = finger_pair(r.c, 0.006);
  core::Grasp one(topology(), {gripper()}, {});
  const auto events = one.update(r.x, closed);
  ASSERT_TRUE(holds(one, r.k, 0)) << "dos dedos a ±6 mm no agarran el nudo";
  ASSERT_EQ(events.size(), 1u);
  EXPECT_GT(events[0].grabbed, 0);
  for (const auto& a : one.anchors(poses(closed))) {
    EXPECT_LE((a.anchor.to - r.x[static_cast<size_t>(a.anchor.node)]).norm(), 1e-5f) << "el anclaje no está en el nudo";
    EXPECT_LE((a.anchor.from - a.anchor.to).norm(), 1e-6f) << "con los dedos quietos el anclaje se mueve";
    EXPECT_EQ(a.body, 0u) << "el nudo no queda atado al primer dedo";
  }
  const auto ajar = finger_pair(r.c, 0.008);  // 4 mm más abierta: menos que release_gap
  for (const auto& e : one.update(r.x, ajar)) EXPECT_EQ(e.released, 0) << "abrir menos que release_gap suelta";
  EXPECT_TRUE(holds(one, r.k, 0)) << "abrir menos que release_gap suelta el nudo";
}

// Un nudo que ya tiene anclaje (el extremo de un orinque) no se agarra
TEST(Grasp, AnchoredNodeIsNotGrabbed)
{
  const Rest r = rest(topology());
  core::Grasp anchored(topology(), {gripper()}, {r.k});
  anchored.update(r.x, finger_pair(r.c, 0.006));
  EXPECT_FALSE(holds(anchored, r.k, 0));
}

// Con dos pinzas, solo la primera lo coge y, al abrirse esta, la otra
TEST(Grasp, FirstGripperWins)
{
  const Rest r = rest(topology());
  const auto closed = finger_pair(r.c, 0.006);
  core::Grasp two(topology(), {gripper(), {0.004, 0.005, {2, 3}}}, {});
  two.update(r.x, both(closed, closed));
  EXPECT_TRUE(holds(two, r.k, 0) && !holds(two, r.k, 1)) << "con dos pinzas no lo coge solo la primera";
  two.update(r.x, both(finger_pair(r.c, 0.02), closed));
  EXPECT_TRUE(!holds(two, r.k, 0) && holds(two, r.k, 1)) << "al abrirse la primera pinza no lo coge la otra";
}

// Un dedo con pose no finita no agarra, no suelta y no deja anclajes no finitos
TEST(Grasp, NonFiniteFingerIsIgnored)
{
  const Rest r = rest(topology());
  const auto closed = finger_pair(r.c, 0.006);
  core::Grasp lost(topology(), {gripper()}, {});
  lost.update(r.x, closed);
  auto broken = closed;
  broken[0].pose.translation().x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(lost.update(r.x, broken).empty()) << "un dedo no finito agarra o suelta";
  EXPECT_TRUE(anchors_finite(lost.anchors(poses(broken))) && anchors_finite(lost.anchors(poses(closed))))
      << "un dedo no finito deja anclajes no finitos";
}

// Dedos que giran sobre una articulación común (el origen de su marco): lo que se abre son sus cajas
TEST(Grasp, HingedFingersReleaseWhenOpening)
{
  const Rest r = rest(topology());
  const Eigen::Vector3d pivot = r.c - Eigen::Vector3d(0.0, 0.0, 0.03);
  const auto hinged = [&](double angle) {
    std::vector<core::Finger> f;
    for (double side : {-1.0, 1.0}) {
      core::RoundedBox box = kFingerBox;
      box.center_offset = Eigen::Vector3d(0.006 * side, 0.0, 0.03);
      f.push_back({at(pivot) * Eigen::Isometry3d(Eigen::AngleAxisd(side * angle, Eigen::Vector3d::UnitY())), box});
    }
    return f;
  };
  core::Grasp jaw(topology(), {gripper()}, {});
  jaw.update(r.x, hinged(0.0));
  EXPECT_TRUE(holds(jaw, r.k, 0)) << "unos dedos con la caja desplazada de su marco no agarran";
  jaw.update(r.x, hinged(0.3));
  EXPECT_FALSE(holds(jaw, r.k, 0)) << "unos dedos que giran sobre su articulación no sueltan al abrirse";
}

// release_gap menor que grip_margin: al soltar, el nudo sigue pellizcado; no se vuelve a agarrar enseguida
TEST(Grasp, ReleasedNodeIsNotRegrabbed)
{
  const Rest r = rest(topology());
  core::Grasp loose(topology(), {gripper(0.004, 0.002)}, {});
  loose.update(r.x, finger_pair(r.c, 0.006));
  const auto slight = finger_pair(r.c, 0.0075);  // 3 mm más abierta: suelta, y el nudo sigue a 2,5 mm de los dos
  loose.update(r.x, slight);
  EXPECT_FALSE(holds(loose, r.k, 0)) << "vuelve a agarrar en el mismo tick lo que acaba de soltar";
  loose.update(r.x, slight);
  EXPECT_FALSE(holds(loose, r.k, 0)) << "vuelve a agarrar lo que soltó sin que la pinza se haya apartado";
}

// La pinza de GripScene con el XPBD de referencia: agarra al cerrarse, lo agarrado sigue a su punto atado, el tirón
// no estira los tramos, al abrirse lo suelta todo y lo soltado no sigue al dedo
TEST(Grasp, PullAndRelease)
{
  const core::GpuLayout L = core::make_gpu_layout(topology(), core::color_links(topology()));
  GripScene scene(topology(), ref_tick(config(), L));
  scene.settle(5.0);
  scene.pinch = scene.nearest_node(net_middle(config()));
  for (int t = 0; t < GripScene::ticks(); ++t) {
    scene.step(t);
    ASSERT_TRUE(scene.finite()) << "un nudo no es finito en el tick " << t;
  }
  std::printf("PullAndRelease: %zu nudos agarrados al cerrar (%zu en algún momento), distancia máx al punto atado "
              "%.3f mm, estiramiento máx %.2f %% (asentada %.2f %%), lo que siguen al dedo %.1f mm\n",
              scene.held_close, scene.ever_held.size(), 1000.f * scene.attach, 100.f * scene.stretch,
              100.f * scene.settled_stretch, 1000.f * scene.followed());
  EXPECT_GT(scene.held_close, 0u) << "la pinza no agarra nada al cerrarse";
  EXPECT_LE(scene.attach, 1e-3f) << "un nudo agarrado se separa de su punto atado";
  EXPECT_LE(scene.stretch, scene.settled_stretch + 0.01f) << "el tirón estira los tramos";
  EXPECT_TRUE(scene.grasp.held(0).empty()) << "al abrirse la pinza no lo suelta todo";
  EXPECT_LT(scene.followed(), 0.05f) << "lo que se agarró sigue al dedo tras soltarlo";
}

// slip: solo suelta con un tirón finito por encima de max_pull, y sin nudos agarrados no hace nada
TEST(Grasp, SlipNeedsHeldNodes)
{
  const Rest r = rest(topology());
  core::Gripper g = gripper();
  g.max_pull = 1.0;
  core::Grasp grasp(topology(), {g}, {});
  EXPECT_TRUE(grasp.slip({1e3}).empty()) << "suelta sin tener nada agarrado";
  grasp.update(r.x, finger_pair(r.c, 0.006));
  ASSERT_TRUE(holds(grasp, r.k, 0));
  EXPECT_TRUE(grasp.slip({0.5}).empty() && holds(grasp, r.k, 0)) << "suelta por debajo de max_pull";
  EXPECT_TRUE(grasp.slip({std::numeric_limits<double>::quiet_NaN()}).empty() && holds(grasp, r.k, 0))
      << "suelta con un tirón no finito";
  const auto events = grasp.slip({2.0});
  ASSERT_EQ(events.size(), 1u);
  EXPECT_GT(events[0].released, 0);
  EXPECT_FALSE(holds(grasp, r.k, 0)) << "no suelta por encima de max_pull";
  grasp.update(r.x, finger_pair(r.c, 0.006));
  EXPECT_FALSE(holds(grasp, r.k, 0)) << "vuelve a agarrar lo que resbaló sin que la pinza se haya apartado";
}

// La pinza de GripScene con max_pull a la mitad del tirón que hace sin límite: el hilo resbala mientras tira y acaba
// la fase de tirar con menos agarrado; sin límite no resbala nunca
TEST(Grasp, SlipReleasesAboveMaxPull)
{
  const core::GpuLayout L = core::make_gpu_layout(topology(), core::color_links(topology()));
  const auto run = [&](double max_pull) {
    core::Gripper g = gripper();
    g.max_pull = max_pull;
    GripScene scene(topology(), ref_tick(config(), L), g, config().solver.substeps);
    scene.settle(5.0);
    scene.pinch = scene.nearest_node(net_middle(config()));
    for (int t = 0; t < GripScene::ticks(); ++t) scene.step(t);
    return std::make_tuple(scene.max_pull_seen, scene.slipped, scene.held_pull, scene.ever_held.size());
  };
  const auto [pull, free_slipped, free_held, free_ever] = run(0.0);
  const auto [weak_pull, slipped, held, ever] = run(0.5 * pull);
  std::printf("SlipReleasesAboveMaxPull: tirón máx sin límite %.3f N (%zu agarrados al acabar de tirar); con max_pull "
              "%.3f N resbalan %d nudos y quedan %zu (tirón máx %.3f N)\n",
              pull, free_held, 0.5 * pull, slipped, held, weak_pull);
  EXPECT_EQ(free_slipped, 0) << "sin max_pull resbala";
  EXPECT_GT(free_held, 0u);
  EXPECT_GT(ever, 0u) << "con max_pull no llega a agarrar nada";
  EXPECT_GT(slipped, 0) << "con max_pull a la mitad del tirón no resbala";
  EXPECT_LT(held, free_held) << "con max_pull acaba de tirar con lo mismo agarrado";
  (void)free_ever;
}

}  // namespace stonefish_net::test
