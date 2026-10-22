#include "stonefish_net/core/net_config.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace stonefish_net::core
{
namespace
{
[[noreturn]] void error(const std::string& campo, const std::string& motivo)
{
  throw std::invalid_argument(campo + ": " + motivo);
}

// Una sección del YAML: lee cada campo por su nombre y valida su rango en el mismo punto.
class Section
{
public:
  // node_ se inicializa por copia: en yaml-cpp, operator= sobre un Node reasigna su contenido en vez de
  // reenlazarlo y, con una clave ausente, lanza InvalidNode en lugar de dejar un nodo nulo comprobable.
  Section(const YAML::Node& parent, const std::string& key, const std::string& name)
  : Section(parent[key], name)
  {
  }
  Section(const YAML::Node& node, const std::string& name) : node_(node), name_(name)
  {
    if (!node_ || !node_.IsMap()) {
      error(name_, "sección ausente");
    }
  }

  Section sub(const std::string& key) const { return Section(node_, key, name_ + "." + key); }

  double num(const std::string& key) const { return read<double>(key, "valor no numérico"); }
  int integer(const std::string& key) const { return read<int>(key, "valor no entero"); }
  bool flag(const std::string& key) const { return read<bool>(key, "valor no booleano"); }
  std::string text(const std::string& key) const { return read<std::string>(key, "valor no textual"); }
  std::vector<std::string> text_list(const std::string& key) const
  {
    return read<std::vector<std::string>>(key, "debe ser una lista de textos");
  }
  Eigen::Vector3d vec3(const std::string& key) const
  {
    const auto v = read<std::vector<double>>(key, "debe ser [x, y, z]");
    if (v.size() != 3) {
      error(field(key), "debe ser [x, y, z]");
    }
    return {v[0], v[1], v[2]};
  }
  Eigen::Vector3d positive_vec3(const std::string& key) const
  {
    const Eigen::Vector3d v = vec3(key);
    if (!(v.minCoeff() > 0.0)) {
      error(field(key), "sus componentes deben ser > 0");
    }
    return v;
  }

  double positive(const std::string& key) const { return check(key, num(key), [](double v) { return v > 0.0; }, "debe ser > 0"); }
  double non_negative(const std::string& key) const
  {
    return check(key, num(key), [](double v) { return v >= 0.0; }, "debe ser >= 0");
  }
  double at_least_one(const std::string& key) const
  {
    return check(key, num(key), [](double v) { return v >= 1.0; }, "debe ser >= 1");
  }
  double unit(const std::string& key) const
  {
    return check(key, num(key), [](double v) { return v >= 0.0 && v <= 1.0; }, "debe estar en [0, 1]");
  }
  double unit_open(const std::string& key) const
  {
    return check(key, num(key), [](double v) { return v >= 0.0 && v < 1.0; }, "debe estar en [0, 1)");
  }
  double open_range(const std::string& key, double lo, double hi) const
  {
    return check(key, num(key), [=](double v) { return v > lo && v < hi; },
                 "debe estar en (" + fmt(lo) + ", " + fmt(hi) + ")");
  }
  int int_at_least(const std::string& key, int lo) const
  {
    const int v = integer(key);
    if (v < lo) {
      error(field(key), "debe ser >= " + std::to_string(lo));
    }
    return v;
  }

  std::string field(const std::string& key) const { return name_ + "." + key; }

private:
  template <typename T>
  T read(const std::string& key, const char* tipo) const
  {
    const YAML::Node n = node_[key];
    if (!n) {
      error(field(key), "campo ausente");
    }
    try {
      return n.as<T>();
    } catch (const YAML::Exception&) {
      error(field(key), tipo);
    }
  }

  template <typename Pred>
  double check(const std::string& key, double v, Pred ok, const std::string& motivo) const
  {
    if (!ok(v)) {
      error(field(key), motivo);
    }
    return v;
  }

  static std::string fmt(double v)
  {
    std::ostringstream ss;
    ss << v;
    return ss.str();
  }

  YAML::Node node_;
  std::string name_;
};

Section root_section(const YAML::Node& root, const std::string& key) { return Section(root, key, key); }
}  // namespace

NetConfig parse_net_config(const std::string& yaml_text)
{
  YAML::Node root;
  try {
    root = YAML::Load(yaml_text);
  } catch (const YAML::Exception& e) {
    error("yaml", e.what());
  }
  NetConfig c{};

  const Section net = root_section(root, "net");
  c.net.width = net.positive("width");
  c.net.height = net.positive("height");
  c.net.bar = net.positive("bar");
  c.net.opening_deg = net.open_range("opening_deg", 0.0, 180.0);
  c.net.twine_diameter = net.positive("twine_diameter");
  c.net.twine_density = net.positive("twine_density");

  const Section pose = root_section(root, "pose");
  c.pose.x = pose.num("x");
  c.pose.y_center = pose.num("y_center");
  c.pose.z_top = pose.num("z_top");

  const Section frame = root_section(root, "frame_rope");
  c.frame_rope.diameter = frame.positive("diameter");
  c.frame_rope.density = frame.positive("density");

  const Section leads = root_section(root, "leads");
  c.leads.spacing = leads.positive("spacing");
  c.leads.mass = leads.non_negative("mass");

  const Section buoys = root_section(root, "buoys");
  c.buoys.spacing = buoys.positive("spacing");
  c.buoys.dropper = buoys.positive("dropper");
  c.buoys.mass = buoys.positive("mass");

  const Section mooring = root_section(root, "mooring");
  c.mooring.floor_z = mooring.num("floor_z");
  c.mooring.upstream = mooring.non_negative("upstream");
  c.mooring.block_height = mooring.positive("block_height");
  for (auto [key, line] : {std::pair{"bottom", &c.mooring.bottom}, std::pair{"top", &c.mooring.top}}) {
    const Section s = mooring.sub(key);
    line->outward = s.non_negative("outward");
    line->slack = s.at_least_one("slack");
  }

  const Section hydro = root_section(root, "hydro");
  c.hydro.cn = hydro.non_negative("cn");
  c.hydro.ct = hydro.non_negative("ct");
  c.hydro.added_mass = hydro.non_negative("added_mass");
  c.hydro.water_density = hydro.positive("water_density");
  c.hydro.current_grid = hydro.positive("current_grid");

  const Section solver = root_section(root, "solver");
  const std::string backend = solver.text("backend");
  if (backend == "gpu") {
    c.solver.backend = NetConfig::Backend::Gpu;
  } else if (backend == "bullet") {
    c.solver.backend = NetConfig::Backend::Bullet;
  } else {
    error(solver.field("backend"), "debe ser gpu o bullet");
  }
  c.solver.iterations = solver.int_at_least("iterations", 1);
  c.solver.substeps = solver.int_at_least("substeps", 1);
  c.solver.compliance = solver.non_negative("compliance");
  c.solver.contact_hardness = solver.unit("contact_hardness");
  c.solver.margin = solver.positive("margin");
  c.solver.damping = solver.unit("damping");
  c.solver.friction = solver.unit("friction");

  const Section damage = root_section(root, "damage");
  c.damage.seed = damage.integer("seed");
  c.damage.tears = damage.int_at_least("tears", 0);
  c.damage.tear_min_meshes = damage.int_at_least("tear_min_meshes", 1);
  c.damage.tear_max_meshes = damage.int_at_least("tear_max_meshes", c.damage.tear_min_meshes);
  c.damage.stub_fraction = damage.unit("stub_fraction");
  c.damage.loose_fraction = damage.unit("loose_fraction");

  const YAML::Node links = root["robot_contact"];
  if (!links || !links.IsSequence()) {
    error("robot_contact", "debe ser una lista (vacía, [], si ningún robot toca la red)");
  }
  for (size_t k = 0; k < links.size(); ++k) {
    const Section s(links[k], "robot_contact[" + std::to_string(k) + "]");
    NetConfig::RobotLink l;
    l.robot = s.text("robot");
    l.link = s.text("link");
    l.box.half_extents = s.positive_vec3("half_extents");
    l.box.rounding = s.non_negative("rounding");
    l.box.center_offset = s.vec3("center_offset");
    l.friction = s.non_negative("friction");
    if (!(l.box.rounding < l.box.half_extents.minCoeff())) {
      error(s.field("rounding"), "debe ser menor que el menor de half_extents");
    }
    c.robot_contact.push_back(l);
  }

  const Section contact_auto = root_section(root, "robot_contact_auto");
  c.robot_contact_auto.enabled = contact_auto.flag("enabled");
  c.robot_contact_auto.rounding_fraction = contact_auto.unit_open("rounding_fraction");
  c.robot_contact_auto.friction = contact_auto.non_negative("friction");

  const Section solid_auto = root_section(root, "solid_contact_auto");
  c.solid_contact_auto.enabled = solid_auto.flag("enabled");
  c.solid_contact_auto.rounding_fraction = solid_auto.unit_open("rounding_fraction");
  c.solid_contact_auto.friction = solid_auto.non_negative("friction");

  const Section sleep = root_section(root, "sleep");
  c.sleep.enabled = sleep.flag("enabled");
  c.sleep.tile_size = sleep.positive("tile_size");
  c.sleep.wake_radius = sleep.positive("wake_radius");
  c.sleep.sleep_radius = sleep.positive("sleep_radius");
  c.sleep.sleep_speed = sleep.positive("sleep_speed");
  c.sleep.settle_time = sleep.non_negative("settle_time");
  c.sleep.wake_shift = sleep.positive("wake_shift");
  if (!(c.sleep.sleep_radius > c.sleep.wake_radius)) {
    error(sleep.field("sleep_radius"), "debe ser mayor que wake_radius");
  }

  // Opcional: los YAML de antes de las pinzas siguen sirviendo
  if (const YAML::Node grippers = root["grippers"]) {
    if (!grippers.IsSequence()) {
      error("grippers", "debe ser una lista");
    }
    for (size_t k = 0; k < grippers.size(); ++k) {
      const Section s(grippers[k], "grippers[" + std::to_string(k) + "]");
      NetConfig::Gripper g;
      g.robot = s.text("robot");
      g.fingers = s.text_list("fingers");
      if (g.fingers.size() < 2) {
        error(s.field("fingers"), "hacen falta al menos dos dedos");
      }
      for (size_t i = 0; i < g.fingers.size(); ++i) {
        for (size_t j = 0; j < i; ++j) {
          if (g.fingers[i] == g.fingers[j]) {
            error(s.field("fingers"), "el dedo '" + g.fingers[i] + "' está repetido");
          }
        }
      }
      g.grip_margin = s.positive("grip_margin");
      g.release_gap = s.positive("release_gap");
      c.grippers.push_back(g);
    }
  }

  c.diagnostics.enabled = root_section(root, "diagnostics").flag("enabled");
  return c;
}

NetConfig load_net_config(const std::string& path)
{
  std::ifstream f(path);
  if (!f) {
    error("fichero", "no se puede abrir '" + path + "'");
  }
  std::stringstream ss;
  ss << f.rdbuf();
  return parse_net_config(ss.str());
}

}  // namespace stonefish_net::core
