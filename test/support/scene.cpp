#include "scene.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <memory>

namespace stonefish_net::test
{

const std::string& config_path()
{
  static const std::string path = NET_TEST_CONFIG;
  return path;
}

const core::NetConfig& config()
{
  static const core::NetConfig cfg = core::load_net_config(config_path());
  return cfg;
}

const core::NetTopology& topology()
{
  static const core::NetTopology topo = core::build_topology(config());
  return topo;
}

core::XpbdParams params(const core::NetConfig& cfg)
{
  const int n = cfg.solver.substeps;
  core::XpbdParams p{};
  p.h = static_cast<float>(kTick / n);
  p.rho = static_cast<float>(cfg.hydro.water_density);
  p.g = 9.81f;
  p.cn = static_cast<float>(cfg.hydro.cn);
  p.ct = static_cast<float>(cfg.hydro.ct);
  p.compliance = static_cast<float>(cfg.solver.compliance);
  p.damping = core::substep_damping(cfg.solver.damping, n);
  p.friction = static_cast<float>(cfg.solver.friction);
  p.floor_z = static_cast<float>(cfg.mooring.floor_z);
  return p;
}

core::XpbdParams dry(const core::NetConfig& cfg)
{
  core::XpbdParams p = params(cfg);
  p.cn = p.ct = 0.f;
  return p;
}

Eigen::Vector3f current_at(const Eigen::Vector3f&) { return kCurrent; }

std::vector<core::Anchor> fixed_buoys(const core::NetTopology& topo)
{
  std::vector<core::Anchor> a;
  for (const auto& b : topo.marks.buoys) {
    const Eigen::Vector3f x = topo.nodes[static_cast<size_t>(b.node)].position.cast<float>();
    a.push_back({b.node, x, x});
  }
  return a;
}

std::vector<int> buoy_nodes(const core::NetTopology& topo)
{
  std::vector<int> nodes;
  for (const auto& b : topo.marks.buoys) nodes.push_back(b.node);
  return nodes;
}

Eigen::Vector3d net_middle(const core::NetConfig& cfg)
{
  return {cfg.pose.x, cfg.pose.y_center, cfg.pose.z_top + 0.5 * cfg.net.height};
}

Eigen::Vector3f center(const std::vector<Eigen::Vector3f>& x)
{
  Eigen::Vector3f c = Eigen::Vector3f::Zero();
  for (const auto& p : x) c += p;
  return c / static_cast<float>(x.size());
}

float max_stretch(const core::NetTopology& topo, const std::vector<Eigen::Vector3f>& x)
{
  float worst = 0.f;
  for (const auto& l : topo.links) {
    const float len = (x[static_cast<size_t>(l.a)] - x[static_cast<size_t>(l.b)]).norm();
    worst = std::max(worst, len / static_cast<float>(l.rest_length) - 1.f);
  }
  return worst;
}

float relative(const Eigen::Vector3d& got, const Eigen::Vector3d& want)
{
  return static_cast<float>((got - want).norm() / std::max(want.norm(), 1e-6));
}

core::GpuLayout hanging_chain(int count)
{
  core::NetTopology topo;
  for (int k = 0; k < count; ++k) {
    core::Node n;
    n.position = Eigen::Vector3d(0.0, 0.0, 0.1 * k);
    n.kind = core::NodeKind::Dropper;
    n.mass = 0.01;
    n.inertial_mass = 0.02;
    topo.nodes.push_back(n);
    if (k > 0) topo.links.push_back({k - 1, k, 0.1, 0.002, core::LinkKind::Dropper});
  }
  return core::make_gpu_layout(topo, core::color_links(topo));
}

core::ContactBody moving_box(const Eigen::Vector3d& c0, const Eigen::Vector3d& c1, const Eigen::Matrix3d& rot)
{
  core::ContactBody box{};
  box.box = core::RoundedBox{Eigen::Vector3d(0.3, 0.25, 0.15), 0.04, Eigen::Vector3d::Zero()};
  box.friction = 0.5f;
  box.pose0 = Eigen::Translation3d(c0) * Eigen::Isometry3d(rot);
  box.pose1 = Eigen::Translation3d(c1) * Eigen::Isometry3d(rot);
  box.cg0 = c0;
  box.cg1 = c1;
  box.v_cg = (c1 - c0) / kTick;
  box.w = Eigen::Vector3d::Zero();
  return box;
}

ExclusionCase exclusion_case(const core::NetConfig& cfg, const core::GpuLayout& L)
{
  const Eigen::Vector3d middle = net_middle(cfg);
  ExclusionCase e{{}, -1};
  double best = 1e9;
  for (size_t k = 0; k < L.node_count(); ++k) {
    const Eigen::Vector3d x(L.x0[4 * k], L.x0[4 * k + 1], L.x0[4 * k + 2]);
    if (L.x0[4 * k + 3] > 0.f && (x - middle).norm() < best) {
      best = (x - middle).norm();
      e.node = static_cast<int>(k);
    }
  }
  const auto k = static_cast<size_t>(e.node);
  // Descentrada 2 mm: el nudo sale por la cara −x
  const Eigen::Vector3d c = Eigen::Vector3d(L.x0[4 * k], L.x0[4 * k + 1], L.x0[4 * k + 2]) + Eigen::Vector3d(0.002, 0.0, 0.0);
  for (int b = 0; b < 3; ++b) {
    core::ContactBody box = moving_box(c, c);
    box.box = core::RoundedBox{Eigen::Vector3d(0.005, 0.005, 0.005), 0.001, Eigen::Vector3d::Zero()};
    if (b < 2) box.excluded_nodes = {e.node};
    e.bodies.push_back(box);
  }
  return e;
}

core::Gripper gripper(double grip_margin, double release_gap) { return {grip_margin, release_gap, {0, 1}}; }

Eigen::Isometry3d at(const Eigen::Vector3d& c) { return Eigen::Translation3d(c) * Eigen::Isometry3d::Identity(); }

std::vector<core::Finger> finger_pair(const Eigen::Vector3d& c, double half_gap)
{
  return {{at(c - Eigen::Vector3d(half_gap, 0.0, 0.0)), kFingerBox}, {at(c + Eigen::Vector3d(half_gap, 0.0, 0.0)), kFingerBox}};
}

NetTick ref_tick(const core::NetConfig& cfg, const core::GpuLayout& L)
{
  struct Ref
  {
    core::XpbdState s;
    core::ContactMemory mem;
    std::vector<core::BodyReaction> reactions;
  };
  auto ref = std::make_shared<Ref>(Ref{core::xpbd_init(L), core::contact_memory(L), {}});
  const core::XpbdParams p = params(cfg);
  const int n = cfg.solver.substeps;
  return [ref, &L, p, n](const std::vector<core::Anchor>& anchors, const std::vector<core::ContactBody>& bodies,
                         std::vector<Eigen::Vector3f>& x, std::vector<Eigen::Vector3f>& anchor_dx) {
    anchor_dx.assign(anchors.size(), Eigen::Vector3f::Zero());
    ref->reactions.assign(bodies.size(), core::BodyReaction{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()});
    for (int i = 0; i < n; ++i) {
      core::xpbd_substep(L, p, current_at, anchors, bodies, i, n, ref->s, ref->mem, anchor_dx, ref->reactions);
    }
    x = ref->s.x;
  };
}

GripScene::GripScene(const core::NetTopology& t, NetTick f)
: topo(t), tick(std::move(f)), grasp(t, {gripper()}, buoy_nodes(t)), buoys(fixed_buoys(t))
{
}

void GripScene::settle(double seconds)
{
  for (int i = 0; i < ticks_in(seconds); ++i) tick(buoys, {}, x, anchor_dx);
  settled_stretch = max_stretch(topo, x);
}

Eigen::Vector3d GripScene::nearest_node(const Eigen::Vector3d& target) const
{
  size_t best = 0;
  for (size_t k = 0; k < x.size(); ++k) {
    if (!topo.nodes[k].fixed && (x[k].cast<double>() - target).norm() < (x[best].cast<double>() - target).norm()) best = k;
  }
  return x[best].cast<double>();
}

int GripScene::ticks() { return ticks_in(kClose + kPull + kStill + kOpen + kAway); }

GripScene::Phase GripScene::phase(int t)
{
  const double e = (t + 1) * kTick - 1e-9;
  if (e < kClose) return Close;
  if (e < kClose + kPull) return Pull;
  if (e < kClose + kPull + kStill) return Still;
  if (e < kClose + kPull + kStill + kOpen) return Open;
  return Away;
}

std::array<Eigen::Vector3d, 2> GripScene::fingers(int t) const
{
  const double e = (t + 1) * kTick;
  const auto ramp = [e](double from, double len) { return std::clamp(e - from, 0.0, len); };
  const double half = 0.03 - 0.02 * ramp(0.0, kClose) + 0.02 * ramp(kClose + kPull + kStill, kOpen);
  const double shift = -0.1 * ramp(kClose, kPull);
  const double away = 0.1 * ramp(kClose + kPull + kStill + kOpen, kAway);
  return {pinch + Eigen::Vector3d(shift - half - away, 0.0, 0.0), pinch + Eigen::Vector3d(shift + half + away, 0.0, 0.0)};
}

namespace
{
core::ContactBody finger_body(const Eigen::Vector3d& c0, const Eigen::Vector3d& c1)
{
  core::ContactBody b{};
  b.box = kFingerBox;
  b.friction = 0.5f;
  b.pose0 = at(c0);
  b.pose1 = at(c1);
  b.cg0 = c0;
  b.cg1 = c1;
  b.v_cg = (c1 - c0) / kTick;
  b.w = Eigen::Vector3d::Zero();
  return b;
}
}  // namespace

void GripScene::step(int t)
{
  const auto c0 = fingers(t - 1), c1 = fingers(t);
  grasp.update(x, {{at(c0[0]), kFingerBox}, {at(c0[1]), kFingerBox}});
  std::vector<core::Anchor> anchors = buoys;
  const std::vector<core::GripAnchor> held = grasp.anchors({at(c1[0]), at(c1[1])});
  for (const auto& h : held) anchors.push_back(h.anchor);
  std::vector<core::ContactBody> bodies{finger_body(c0[0], c1[0]), finger_body(c0[1], c1[1])};
  grasp.exclude_held(bodies);
  if (phase(t) == Away && away_start.empty()) away_start = x;
  tick(anchors, bodies, x, anchor_dx);

  for (size_t j = 0; j < held.size(); ++j) {
    const core::Anchor& a = held[j].anchor;
    ever_held.insert(a.node);
    grip_dx += anchor_dx[buoys.size() + j].cast<double>();
    if (phase(t) == Pull) attach = std::max(attach, (x[static_cast<size_t>(a.node)] - a.to).norm());
  }
  if (first_held.empty()) first_held = grasp.held(0);
  if (phase(t) == Close) held_close = grasp.held(0).size();
  stretch = std::max(stretch, max_stretch(topo, x));
  if (std::getenv("GPU_CHECK_TRACE") && (t % 20 == 19 || phase(t) != phase(t + 1) || max_stretch(topo, x) > 0.01f)) {
    const Eigen::Vector3f nx = ever_held.empty() ? Eigen::Vector3f::Zero() : x[static_cast<size_t>(*ever_held.begin())];
    std::printf("  t %4d fase %d: agarrados %zu, nudo (%.4f, %.4f, %.4f), dedos x %.4f %.4f, estira %.2f %%\n", t, phase(t),
                grasp.held(0).size(), nx.x(), nx.y(), nx.z(), c1[0].x(), c1[1].x(), 100.f * max_stretch(topo, x));
  }
}

bool GripScene::finite() const
{
  return std::all_of(x.begin(), x.end(), [](const Eigen::Vector3f& p) { return p.allFinite(); });
}

float GripScene::followed() const
{
  float worst = 0.f;
  for (int k : ever_held) worst = std::max(worst, away_start[static_cast<size_t>(k)].x() - x[static_cast<size_t>(k)].x());
  return worst;
}

}  // namespace stonefish_net::test
