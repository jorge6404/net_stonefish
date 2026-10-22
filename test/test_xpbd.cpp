// El subpaso XPBD de referencia (core/xpbd_ref) y el acoplamiento con los sólidos (core/xpbd), con la red de
// test/net_test.yaml y ticks de 5 ms (200 Hz). Las pruebas con la red completa tardan unos minutos.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "stonefish_net/core/current_grid.hpp"
#include "support/scene.hpp"

namespace stonefish_net::test
{
namespace
{
core::GpuLayout net_layout() { return core::make_gpu_layout(topology(), core::color_links(topology())); }

// La red asentada `seconds` s con la corriente del tanque y las boyas fijas
core::XpbdState settled(const core::GpuLayout& L, double seconds, core::ContactMemory& mem)
{
  const core::XpbdParams p = params(config());
  const int n = config().solver.substeps;
  const auto anchors = fixed_buoys(topology());
  core::XpbdState s = core::xpbd_init(L);
  std::vector<Eigen::Vector3f> anchor_dx;
  std::vector<core::BodyReaction> reactions;
  for (int t = 0; t < ticks_in(seconds); ++t) {
    for (int i = 0; i < n; ++i) core::xpbd_substep(L, p, current_at, anchors, {}, i, n, s, mem, anchor_dx, reactions);
  }
  return s;
}

// Lo más hundido de la red en la caja de un cuerpo: nudos y punto más hundido de cada tramo
double deepest_in(const core::ContactBody& box, const core::GpuLayout& L, const std::vector<Eigen::Vector3f>& x)
{
  double worst = 0.0;
  for (const auto& p : x) {
    if (const auto hit = core::box_contact(box.box, box.pose1, p.cast<double>())) worst = std::max(worst, hit->depth);
  }
  for (size_t i = 0; i < L.link_count(); ++i) {
    const Eigen::Vector3d a = x[static_cast<size_t>(L.links[2 * i])].cast<double>();
    const Eigen::Vector3d b = x[static_cast<size_t>(L.links[2 * i + 1])].cast<double>();
    if (!core::segment_in_box_bounds(box.box, box.pose1, a, b)) continue;
    const double u = core::deepest_on_segment(box.box, box.pose1, a, b);
    if (const auto hit = core::box_contact(box.box, box.pose1, a + u * (b - a))) worst = std::max(worst, hit->depth);
  }
  return worst;
}
}  // namespace

// La inversa de la masa efectiva de un sólido en un punto: un impulso f en ese punto le cambia la velocidad ahí en
// f/m + (I⁻¹ (r × f)) × r
TEST(Xpbd, PointInvMass)
{
  const double mass = 2.0;
  const Eigen::Matrix3d R = Eigen::AngleAxisd(0.7, Eigen::Vector3d(1.0, 2.0, 3.0).normalized()).toRotationMatrix();
  const Eigen::Matrix3d inv_inertia = R * Eigen::Vector3d(10.0, 5.0, 1.0 / 0.3).asDiagonal() * R.transpose();
  const Eigen::Vector3d r(0.1, -0.2, 0.3), f(1.0, 2.0, -0.5);
  const Eigen::Vector3d want = f / mass + (inv_inertia * r.cross(f)).cross(r);
  const Eigen::Vector3d got = core::point_inv_mass(mass, inv_inertia, r).cast<double>() * f;
  EXPECT_LE((got - want).norm(), 1e-5 * want.norm()) << "no da la velocidad del punto";
}

// La rejilla de corriente cubre la red con una celda de margen, y se queda acotada aunque un nudo salga disparado o
// no sea finito
TEST(Xpbd, CurrentGrid)
{
  std::vector<Eigen::Vector3f> x{{0.f, 0.f, 0.f}, {1.f, 6.f, 4.f}};
  const core::CurrentGrid g = core::current_grid_for(x, 0.5f);
  EXPECT_TRUE(g.dims[0] == 5 && g.dims[1] == 15 && g.dims[2] == 11 && g.origin.isApprox(Eigen::Vector3f(-0.5f, -0.5f, -0.5f)))
      << "la rejilla no cubre la red con una celda de margen";
  x.push_back(Eigen::Vector3f(std::nanf(""), 0.f, 0.f));
  x.push_back(Eigen::Vector3f(1000.f, -1000.f, 3.f));
  const core::CurrentGrid wild = core::current_grid_for(x, 0.5f);
  EXPECT_TRUE(wild.dims[0] <= core::kMaxCurrentGrid && wild.dims[1] <= core::kMaxCurrentGrid &&
              wild.dims[2] <= core::kMaxCurrentGrid && wild.origin.allFinite() && wild.cell >= 0.5f &&
              wild.origin.x() <= 0.f && wild.origin.x() + wild.cell * (wild.dims[0] - 1) >= 1000.f)
      << "la rejilla no queda acotada";
}

// Un nudo que excluye dos cuerpos no toca ninguno de los dos y sí el tercero; con 33 cuerpos que lo excluyen, la
// máscara tiene sus 32 bits y el 33.º no cabe
TEST(Xpbd, ExclusionMask)
{
  const core::GpuLayout L = net_layout();
  const ExclusionCase e = exclusion_case(config(), L);
  core::XpbdState s = core::xpbd_init(L);
  core::ContactMemory mem = core::contact_memory(L);
  std::vector<Eigen::Vector3f> anchor_dx;
  std::vector<core::BodyReaction> r;
  core::xpbd_substep(L, params(config()), current_at, {}, e.bodies, 0, config().solver.substeps, s, mem, anchor_dx, r);
  EXPECT_EQ(r[0].momentum.norm(), 0.0) << "el nudo " << e.node << " toca un cuerpo que excluye";
  EXPECT_EQ(r[1].momentum.norm(), 0.0) << "el nudo " << e.node << " toca un cuerpo que excluye";
  EXPECT_GT(r[2].momentum.norm(), 0.0) << "el nudo no toca el cuerpo que no excluye";

  std::vector<core::ContactBody> many(33);
  for (auto& b : many) b.excluded_nodes = {0};
  EXPECT_EQ(core::exclusion_masks(many, 2), (std::vector<uint32_t>{0xFFFFFFFFu, 0u}))
      << "la máscara de 33 cuerpos no es la de los 32 primeros";
  many.assign(34, core::ContactBody{});
  many[5].excluded_nodes = {0};
  many[33].excluded_nodes = {1};
  EXPECT_EQ(core::cannot_exclude(many), std::vector<size_t>{33})
      << "no detecta el cuerpo 33, que excluye nudos y no cabe en la máscara";
}

// Una cadena colgada de un ancla, asentada: la fuerza sobre la boya (reaction_force de lo que movió el ancla en
// un tick) es el peso de toda la cadena, hacia abajo (+z en NED)
TEST(Xpbd, AnchorCarriesHangingWeight)
{
  const int count = 5, n = config().solver.substeps;
  const core::GpuLayout L = hanging_chain(count);
  const core::XpbdParams p = dry(config());
  const std::vector<core::Anchor> anchors{{0, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero()}};
  core::XpbdState s = core::xpbd_init(L);
  core::ContactMemory mem = core::contact_memory(L);
  std::vector<core::BodyReaction> reactions;
  std::vector<Eigen::Vector3f> moved;
  const auto still = [](const Eigen::Vector3f&) { return Eigen::Vector3f::Zero(); };
  for (int t = 0; t < ticks_in(3.0); ++t) {
    moved.assign(1, Eigen::Vector3f::Zero());
    for (int i = 0; i < n; ++i) core::xpbd_substep(L, p, still, anchors, {}, i, n, s, mem, moved, reactions);
  }
  const Eigen::Vector3d force = core::reaction_force(moved[0].cast<double>(), kTick, n);
  const double weight = count * 0.01 * p.g;
  EXPECT_NEAR(force.z(), weight, 0.02 * weight) << "la reacción del ancla no es el peso colgado";
  EXPECT_LE(force.head<2>().norm(), 0.02 * weight) << "la reacción del ancla no es vertical";
}

// Un ancla que se mueve a 0,1 m/s: dentro del tick sigue al ojal subpaso a subpaso, así que su nudo va a 0,1 m/s y no
// a saltos de n veces eso
TEST(Xpbd, MovingAnchorIsSmooth)
{
  const int n = config().solver.substeps;
  const core::GpuLayout L = hanging_chain(5);
  const core::XpbdParams p = dry(config());
  const Eigen::Vector3f u(0.1f, 0.f, 0.f);
  core::XpbdState s = core::xpbd_init(L);
  core::ContactMemory mem = core::contact_memory(L);
  std::vector<core::BodyReaction> reactions;
  std::vector<Eigen::Vector3f> moved;
  const auto still = [](const Eigen::Vector3f&) { return Eigen::Vector3f::Zero(); };
  Eigen::Vector3f eye = Eigen::Vector3f::Zero();
  float fastest = 0.f;
  for (int t = 0; t < ticks_in(2.0); ++t) {
    const Eigen::Vector3f next = eye + static_cast<float>(kTick) * u;
    const std::vector<core::Anchor> anchors{{0, eye, next}};
    for (int i = 0; i < n; ++i) {
      core::xpbd_substep(L, p, still, anchors, {}, i, n, s, mem, moved, reactions);
      if (t >= ticks_in(1.0)) fastest = std::max(fastest, s.v[0].norm());
    }
    eye = next;
  }
  EXPECT_LE(fastest, 1.5f * u.norm()) << "el nudo anclado va a saltos";
}

// Un tramo degenerado (dos nudos en el mismo punto) no da NaN
TEST(Xpbd, DegenerateLinkStaysFinite)
{
  core::GpuLayout D = net_layout();
  const auto a = static_cast<size_t>(D.links[0]), b = static_cast<size_t>(D.links[1]);
  for (size_t c = 0; c < 3; ++c) D.x0[4 * b + c] = D.x0[4 * a + c];
  core::XpbdState s = core::xpbd_init(D);
  core::ContactMemory mem = core::contact_memory(D);
  std::vector<Eigen::Vector3f> anchor_dx;
  std::vector<core::BodyReaction> reactions;
  core::xpbd_substep(D, params(config()), current_at, fixed_buoys(topology()), {}, 0, 1, s, mem, anchor_dx, reactions);
  for (const auto& x : s.x) ASSERT_TRUE(x.allFinite()) << "posición no finita";
}

// La red se asienta en 15 s con la corriente y las boyas fijas: sin estirarse, sobre el suelo y llevada en +x.
// 15 s: solver.damping es por tick, y a 200 Hz frena el doble por segundo que a 100 Hz; la red llega más despacio
// (sin oscilar) a donde la deja la corriente
TEST(Xpbd, NetSettles)
{
  const core::GpuLayout L = net_layout();
  const core::XpbdParams p = params(config());
  const int n = config().solver.substeps;
  const auto anchors = fixed_buoys(topology());
  core::XpbdState s = core::xpbd_init(L);
  core::ContactMemory mem = core::contact_memory(L);
  std::vector<Eigen::Vector3f> anchor_dx;
  std::vector<core::BodyReaction> reactions;
  const Eigen::Vector3f c0 = center(s.x);
  const int ticks = ticks_in(15.0);
  float vmax_last = 0.f;
  for (int t = 0; t < ticks; ++t) {
    for (int i = 0; i < n; ++i) core::xpbd_substep(L, p, current_at, anchors, {}, i, n, s, mem, anchor_dx, reactions);
    for (size_t k = 0; k < s.x.size(); ++k) {
      ASSERT_TRUE(s.x[k].allFinite() && s.v[k].allFinite()) << "nudo " << k << " no finito en el tick " << t;
      if (t >= ticks - ticks_in(1.0)) vmax_last = std::max(vmax_last, s.v[k].norm());
    }
  }
  float stretch = 0.f, deepest = -1e9f;
  for (size_t i = 0; i < L.link_count(); ++i) {
    const float len = (s.x[static_cast<size_t>(L.links[2 * i + 1])] - s.x[static_cast<size_t>(L.links[2 * i])]).norm();
    stretch = std::max(stretch, len / L.link_props[4 * i] - 1.f);
  }
  for (const auto& x : s.x) deepest = std::max(deepest, x.z());
  const Eigen::Vector3f c1 = center(s.x);
  std::printf("NetSettles: |v| máx último segundo %.4f m/s, estiramiento máx %.3f %%, z máx %.4f m (suelo %.3f), "
              "centro x %+.3f m\n", vmax_last, 100.f * stretch, deepest, p.floor_z, c1.x() - c0.x());
  EXPECT_LT(vmax_last, 0.05f) << "la red no se asienta en 15 s";
  if (config().solver.compliance == 0.0) {
    EXPECT_LT(stretch, 0.02f) << "los tramos se estiran más de un 2 %";
  }
  EXPECT_LE(deepest, p.floor_z + 1e-4f) << "hay nudos por debajo del suelo";
  EXPECT_GT(c1.x(), c0.x()) << "la corriente no desplaza la red en +x";
}

// La red, asentada 3 s, y una caja de 0,6 × 0,5 × 0,3 m que avanza 0,3 m en +x a `speed` contra ella por el centro y
// luego se para 0,4 s. Al final de cada tick ningún nudo ni punto de tramo queda dentro de la caja (más de 1 mm) y la
// red recibe cantidad de movimiento en +x (empuja a la caja hacia atrás).
class BoxPush : public ::testing::TestWithParam<double>
{
};

TEST_P(BoxPush, NetStaysOutAndPushesBack)
{
  const double speed = GetParam();
  const core::GpuLayout L = net_layout();
  const core::XpbdParams p = params(config());
  const int n = config().solver.substeps;
  const auto anchors = fixed_buoys(topology());
  core::ContactMemory mem = core::contact_memory(L);
  core::XpbdState s = settled(L, 3.0, mem);
  std::vector<Eigen::Vector3f> anchor_dx;
  std::vector<core::BodyReaction> reactions;

  // Altura media de la red, y su x en el centro
  const double z_mid = config().pose.z_top + 0.5 * config().net.height;
  double x_net = 0.0;
  int count = 0;
  for (const auto& x : s.x) {
    if (std::abs(x.y() - config().pose.y_center) < 0.3 && std::abs(x.z() - z_mid) < 0.2) {
      x_net += x.x();
      ++count;
    }
  }
  x_net /= std::max(count, 1);

  Eigen::Vector3d c(x_net - 0.35, config().pose.y_center, z_mid);
  const int moving = static_cast<int>(std::lround(0.3 / speed / kTick)), ticks = moving + ticks_in(0.4);
  Eigen::Vector3d momentum = Eigen::Vector3d::Zero();
  double worst = 0.0;
  for (int t = 0; t < ticks; ++t) {
    const Eigen::Vector3d c1 = c + Eigen::Vector3d(t < moving ? speed * kTick : 0.0, 0.0, 0.0);
    core::ContactBody box = moving_box(c, c1);
    box.v_cg = Eigen::Vector3d(t < moving ? speed : 0.0, 0.0, 0.0);
    c = c1;
    reactions.assign(1, core::BodyReaction{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()});
    for (int i = 0; i < n; ++i) core::xpbd_substep(L, p, current_at, anchors, {box}, i, n, s, mem, anchor_dx, reactions);
    momentum += reactions[0].momentum;
    worst = std::max(worst, deepest_in(box, L, s.x));
  }
  std::printf("BoxPush %.1f m/s: penetración máx %.2f mm, Σ m·Δx de la red (%+.4f, %+.4f, %+.4f) kg·m\n", speed,
              1000.0 * worst, momentum.x(), momentum.y(), momentum.z());
  EXPECT_LE(worst, 1e-3) << "la red queda dentro de la caja";
  EXPECT_GT(momentum.x(), 0.0) << "la red no empuja la caja";
}

INSTANTIATE_TEST_SUITE_P(Speeds, BoxPush, ::testing::Values(0.5, 2.0));

// La red con boyas que se mueven, como en Stonefish: cada boya es un punto de 0,5 kg (la masa del YAML) con el
// empuje del asset (6,27 l, medio sumergido cuanto más sale del agua) y arrastre cuadrático, integrado tick a tick
// como lo haría Bullet, y acoplado a la red como en GpuSolver: el ancla conoce la masa de la boya
// (Anchor::inv_mass) y, al acabar el tick, la boya se pone donde la red dejó el ojal, con la velocidad de lo que
// se movió en el tick. Con un ancla de masa infinita y la reacción en el tick siguiente, lo de kilos de red y plomos
// sobre 0,5 kg se pasa de largo cada tick y diverge. En 10 s nada deja de ser finito y en el último segundo las
// boyas van a menos de 0,1 m/s.
TEST(Xpbd, BuoysSettle)
{
  const core::NetConfig& cfg = config();
  const core::NetTopology& topo = topology();
  const core::GpuLayout L = net_layout();
  const core::XpbdParams p = params(cfg);
  const int n = cfg.solver.substeps;
  const double mass = cfg.buoys.mass, volume = 6.2738e-3, half_height = 0.151, radius = 0.113;
  const double area = M_PI * radius * radius, rho = cfg.hydro.water_density, g = p.g;
  const size_t count = topo.marks.buoys.size();
  std::vector<Eigen::Vector3d> eye(count), vel(count, Eigen::Vector3d::Zero());
  for (size_t k = 0; k < count; ++k) eye[k] = topo.nodes[static_cast<size_t>(topo.marks.buoys[k].node)].position;
  core::XpbdState s = core::xpbd_init(L);
  core::ContactMemory mem = core::contact_memory(L);
  std::vector<core::BodyReaction> reactions;
  std::vector<Eigen::Vector3f> moved;
  double vmax_last = 0.0;
  const int ticks = ticks_in(10.0);
  for (int t = 0; t < ticks; ++t) {
    std::vector<core::Anchor> anchors;
    for (size_t k = 0; k < count; ++k) {
      // El ojal está abajo: el centro de la boya, half_height más arriba (−z)
      const double depth = eye[k].z() - half_height;
      const double wet = std::clamp(depth / (2.0 * half_height) + 0.5, 0.0, 1.0);
      const Eigen::Vector3d drag = -0.5 * rho * area * vel[k].norm() * vel[k];
      const Eigen::Vector3d f = drag + Eigen::Vector3d(0.0, 0.0, (mass - rho * volume * wet) * g);
      const Eigen::Vector3f from = eye[k].cast<float>();
      vel[k] += kTick * f / mass;
      eye[k] += kTick * vel[k];
      core::Anchor a{topo.marks.buoys[k].node, from, eye[k].cast<float>()};
      a.inv_mass = core::point_inv_mass(mass, Eigen::Matrix3d::Zero(), Eigen::Vector3d::Zero());
      anchors.push_back(a);
    }
    moved.assign(count, Eigen::Vector3f::Zero());
    for (int i = 0; i < n; ++i) core::xpbd_substep(L, p, current_at, anchors, {}, i, n, s, mem, moved, reactions);
    for (size_t k = 0; k < count; ++k) {
      // Como GpuSolver: la boya, donde la red dejó el ojal y con la velocidad de lo que se movió en el tick
      const Eigen::Vector3d start = anchors[k].from.cast<double>(), end = eye[k] + s.anchor_offset[k].cast<double>();
      const Eigen::Matrix3d inv_mass = anchors[k].inv_mass.cast<double>();
      vel[k] += inv_mass * core::eye_impulse(inv_mass, (end - start) / kTick - vel[k]);
      eye[k] = end;
      ASSERT_TRUE(eye[k].allFinite()) << "boya " << k << " no finita en el tick " << t;
      if (t >= ticks - ticks_in(1.0)) vmax_last = std::max(vmax_last, vel[k].norm());
    }
    for (size_t k = 0; k < s.x.size(); ++k) ASSERT_TRUE(s.x[k].allFinite()) << "nudo " << k << " no finito en el tick " << t;
  }
  std::printf("BuoysSettle: |v| máx de las boyas en el último segundo %.4f m/s, boya 0 en z %.3f m\n", vmax_last, eye[0].z());
  EXPECT_LT(vmax_last, 0.1) << "las boyas no se asientan";
}

// El ROV (caja de 0,58 × 0,48 × 0,28 m y 11,3 kg) empuja la red asentada con 40 N durante 3 s y luego la suelta:
// se mueve como lo haría Bullet, con la reacción de la red del tick anterior (SolidContacts) y arrastre
// cuadrático. Nada deja de ser finito, no atraviesa la red más de 1 cm y nunca pasa de 1 m/s.
TEST(Xpbd, RovPushesWithoutTunneling)
{
  const core::NetConfig& cfg = config();
  const core::GpuLayout L = net_layout();
  const core::XpbdParams p = params(cfg);
  const int n = cfg.solver.substeps;
  const auto anchors = fixed_buoys(topology());
  core::ContactMemory mem = core::contact_memory(L);
  core::XpbdState s = settled(L, 3.0, mem);
  std::vector<Eigen::Vector3f> anchor_dx;
  std::vector<core::BodyReaction> reactions;
  const double mass = 11.27, thrust = 40.0, area = 0.48 * 0.28, rho = cfg.hydro.water_density;
  const double z_mid = cfg.pose.z_top + 0.5 * cfg.net.height;
  double x_net = 0.0;
  for (const auto& x : s.x) x_net = std::max(x_net, static_cast<double>(x.x()));
  core::ContactBody box{};
  box.box = core::RoundedBox{Eigen::Vector3d(0.29, 0.24, 0.14), 0.04, Eigen::Vector3d::Zero()};
  box.friction = 0.5f;
  box.w = Eigen::Vector3d::Zero();
  Eigen::Vector3d c(x_net - 1.0, cfg.pose.y_center, z_mid), v = Eigen::Vector3d::Zero(), force = Eigen::Vector3d::Zero();
  double worst = 0.0, fastest = 0.0;
  for (int t = 0; t < ticks_in(5.0); ++t) {
    const Eigen::Vector3d push(t < ticks_in(3.0) ? thrust : 0.0, 0.0, 0.0);
    box.pose0 = at(c);
    v += kTick * (push + force - 0.5 * rho * area * v.norm() * v) / mass;
    c += kTick * v;
    box.pose1 = at(c);
    box.cg0 = box.pose0.translation();
    box.cg1 = box.pose1.translation();
    box.v_cg = v;
    reactions.assign(1, core::BodyReaction{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()});
    for (int i = 0; i < n; ++i) core::xpbd_substep(L, p, current_at, anchors, {box}, i, n, s, mem, anchor_dx, reactions);
    force = core::reaction_force(reactions[0].momentum, kTick, n);
    ASSERT_TRUE(c.allFinite()) << "el ROV no es finito en el tick " << t;
    for (const auto& x : s.x) {
      ASSERT_TRUE(x.allFinite()) << "un nudo no es finito en el tick " << t;
      if (const auto hit = core::box_contact(box.box, box.pose1, x.cast<double>())) worst = std::max(worst, hit->depth);
    }
    fastest = std::max(fastest, v.norm());
  }
  std::printf("RovPush: |v| máx del ROV %.3f m/s, penetración máx de un nudo %.2f mm, x final %+.3f m respecto a la red\n",
              fastest, 1000.0 * worst, c.x() - x_net);
  EXPECT_LE(worst, 0.01) << "el ROV atraviesa la red";
  EXPECT_LE(fastest, 1.0) << "la reacción de la red lanza al ROV";
}

}  // namespace stonefish_net::test
