// Lectura y validación de config/net.yaml (core::parse_net_config), sobre test/net_test.yaml
#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include "support/scene.hpp"

namespace stonefish_net::test
{
namespace
{
std::string yaml_text()
{
  std::ifstream f(config_path());
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

// El YAML de prueba con `from` cambiado por `to`
std::string replaced(const std::string& from, const std::string& to)
{
  std::string text = yaml_text();
  const size_t at = text.find(from);
  if (at == std::string::npos) throw std::logic_error("el YAML de prueba no tiene '" + from + "'");
  return text.replace(at, from.size(), to);
}

// Si parse_net_config rechaza el texto con un error que empieza por `field`
bool rejects(const std::string& text, const std::string& field)
{
  try {
    core::parse_net_config(text);
  } catch (const std::invalid_argument& e) {
    return std::string(e.what()).rfind(field, 0) == 0;
  }
  return false;
}

std::string with_grippers(const std::string& section) { return replaced("\ngrippers: []\n", "\n" + section + "\n"); }
}  // namespace

// La sección grippers: tal cual (vacía), sin ella (también vacía), con una pinza, y los errores de validación
TEST(Config, Grippers)
{
  EXPECT_TRUE(core::parse_net_config(yaml_text()).grippers.empty()) << "'grippers: []' no da una lista vacía";
  EXPECT_TRUE(core::parse_net_config(with_grippers("")).grippers.empty()) << "sin grippers no da una lista vacía";
  const auto one =
      core::parse_net_config(with_grippers("grippers: [{robot: r, fingers: [a, b], grip_margin: 0.004, release_gap: 0.005}]"))
          .grippers;
  ASSERT_EQ(one.size(), 1u);
  EXPECT_EQ(one[0].robot, "r");
  EXPECT_EQ(one[0].fingers, (std::vector<std::string>{"a", "b"}));
  EXPECT_EQ(one[0].grip_margin, 0.004);
  EXPECT_EQ(one[0].release_gap, 0.005);
  EXPECT_TRUE(rejects(with_grippers("grippers: [{robot: r, fingers: [a], grip_margin: 0.004, release_gap: 0.005}]"),
                      "grippers[0].fingers")) << "acepta una pinza de un solo dedo";
  EXPECT_TRUE(rejects(with_grippers("grippers: [{robot: r, fingers: [a, a], grip_margin: 0.004, release_gap: 0.005}]"),
                      "grippers[0].fingers")) << "acepta dedos repetidos";
  EXPECT_TRUE(rejects(with_grippers("grippers: [{robot: r, fingers: [a, b], grip_margin: 0, release_gap: 0.005}]"),
                      "grippers[0].grip_margin")) << "acepta grip_margin = 0";
  EXPECT_TRUE(rejects(with_grippers("grippers: {robot: r}"), "grippers")) << "acepta grippers que no es una lista";
  // max_pull: opcional (0 = sin límite) y >= 0
  EXPECT_EQ(one[0].max_pull, 0.0) << "sin max_pull no da 0";
  EXPECT_EQ(core::parse_net_config(with_grippers(
                "grippers: [{robot: r, fingers: [a, b], grip_margin: 0.004, release_gap: 0.005, max_pull: 5}]"))
                .grippers[0].max_pull, 5.0);
  EXPECT_TRUE(rejects(with_grippers("grippers: [{robot: r, fingers: [a, b], grip_margin: 0.004, release_gap: 0.005, "
                                    "max_pull: -1}]"), "grippers[0].max_pull")) << "acepta max_pull < 0";
}

TEST(Config, Backend)
{
  EXPECT_EQ(config().solver.backend, core::NetConfig::Backend::Gpu);
  EXPECT_EQ(core::parse_net_config(replaced("{backend: gpu,", "{backend: bullet,")).solver.backend,
            core::NetConfig::Backend::Bullet);
  EXPECT_TRUE(rejects(replaced("{backend: gpu,", "{backend: cpu,"), "solver.backend"));
}

// Los campos de la física nueva: tal cual en la red de prueba (valores neutros) y sus errores de validación
TEST(Config, Physics)
{
  const core::NetConfig& c = config();
  EXPECT_TRUE(std::isinf(c.net.twine_modulus) && std::isinf(c.frame_rope.modulus));
  EXPECT_EQ(c.net.knot_diameter, 0.0);
  EXPECT_EQ(c.hydro.viscosity, 0.0);
  EXPECT_EQ(c.solver.damping_time, 0.0);
  EXPECT_FALSE(c.thruster_jets.enabled);
  EXPECT_EQ(core::parse_net_config(replaced("twine_modulus: .inf", "twine_modulus: 1.0e9")).net.twine_modulus, 1e9);
  EXPECT_TRUE(rejects(replaced("twine_modulus: .inf", "twine_modulus: 0"), "net.twine_modulus"));
  EXPECT_TRUE(rejects(replaced("knot_diameter: 0", "knot_diameter: -0.001"), "net.knot_diameter"));
  EXPECT_TRUE(rejects(replaced("modulus: .inf}", "modulus: 0}"), "frame_rope.modulus"));
  EXPECT_TRUE(rejects(replaced("viscosity: 0", "viscosity: -1"), "hydro.viscosity"));
  EXPECT_TRUE(rejects(replaced("damping_time: 0", "damping_time: -1"), "solver.damping_time"));
  EXPECT_TRUE(rejects(replaced("spread: 0.1", "spread: 0"), "thruster_jets.spread"));
  EXPECT_TRUE(rejects(replaced("\nthruster_jets:", "\nchorros:"), "thruster_jets"));
}

// buoys.enabled: opcional (con boyas); sin boyas no hacen falta sus otros campos
TEST(Config, Buoys)
{
  EXPECT_TRUE(config().buoys.enabled);
  const std::string buoys = "buoys:      {spacing: 2.0, dropper: 0.20, mass: 0.5}";
  EXPECT_FALSE(core::parse_net_config(replaced(buoys, "buoys: {enabled: false}")).buoys.enabled);
  EXPECT_TRUE(core::parse_net_config(replaced(buoys, "buoys: {enabled: true, spacing: 2.0, dropper: 0.20, mass: 0.5}"))
                  .buoys.enabled);
  EXPECT_TRUE(rejects(replaced(buoys, "buoys: {enabled: true}"), "buoys.spacing")) << "con boyas acepta que falten campos";
  EXPECT_TRUE(rejects(replaced(buoys, "buoys: {enabled: quizas}"), "buoys.enabled"));
}

// Destino de cada línea de fondeo: floor sin `to` (la red de prueba), wall con su pared, none, y los errores
TEST(Config, Mooring)
{
  using To = core::NetConfig::LineTo;
  EXPECT_EQ(config().mooring.bottom.to, To::Floor);
  EXPECT_EQ(config().mooring.top.to, To::Floor);
  const std::string bottom = "bottom: {outward: 0.5, slack: 1.05}", top = "top: {outward: 1.0, slack: 1.0}";
  const std::string walls = "floor_z: 4.8, wall_y: 4.0, wall_z: 0.35,\n bottom: {to: none}, top: {to: wall, slack: 1.0}}";
  const std::string text = replaced("floor_z: 4.8, upstream: 1.0, block_height: 0.20,\n             " + bottom + ", " +
                                        top + "}",
                                    walls);
  const core::NetConfig c = core::parse_net_config(text);
  EXPECT_EQ(c.mooring.bottom.to, To::None);
  EXPECT_EQ(c.mooring.top.to, To::Wall);
  EXPECT_EQ(c.mooring.top.slack, 1.0);
  EXPECT_EQ(c.mooring.wall_y, 4.0);
  EXPECT_EQ(c.mooring.wall_z, 0.35);
  EXPECT_TRUE(rejects(replaced(top, "top: {to: techo, slack: 1.0}"), "mooring.top.to")) << "acepta un destino raro";
  EXPECT_TRUE(rejects(replaced(top, "top: {to: wall, slack: 1.0}"), "mooring.wall_y")) << "wall sin wall_y";
  EXPECT_TRUE(rejects(replaced(top, "top: {to: wall}"), "mooring.top.slack")) << "wall sin slack";
  std::string flat = text;
  flat.replace(flat.find("wall_y: 4.0"), 11, "wall_y: 0.0");
  EXPECT_TRUE(rejects(flat, "mooring.wall_y")) << "acepta wall_y = 0";
}

}  // namespace stonefish_net::test
