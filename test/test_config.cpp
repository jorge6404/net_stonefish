// Lectura y validación de config/net.yaml (core::parse_net_config), sobre test/net_test.yaml
#include <gtest/gtest.h>

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
}

TEST(Config, Backend)
{
  EXPECT_EQ(config().solver.backend, core::NetConfig::Backend::Gpu);
  EXPECT_EQ(core::parse_net_config(replaced("{backend: gpu,", "{backend: bullet,")).solver.backend,
            core::NetConfig::Backend::Bullet);
  EXPECT_TRUE(rejects(replaced("{backend: gpu,", "{backend: cpu,"), "solver.backend"));
}

}  // namespace stonefish_net::test
