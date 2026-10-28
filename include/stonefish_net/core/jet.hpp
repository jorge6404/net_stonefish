// Chorro de un propulsor sobre la red, sin ROS ni Stonefish. Las mismas cuentas, en float, que water_at en
// shaders/xpbd_common.glsl.
//
// La estela de la hélice por la teoría de cantidad de movimiento (disco actuador de área A = π D²/4): lejos de la
// hélice el agua sale a u₀ = √(2|T|/(ρA)) por un chorro contraído de radio r₀ = D/(2√2). Después, un chorro redondo
// turbulento: un núcleo de kJetCore diámetros (2 r₀) a u₀ y luego u₀ · x_núcleo/x, con perfil gaussiano exp(−(r/b)²)
// y b = max(r₀, spread · x). Nada aguas arriba de la hélice ni más allá de kJetLength diámetros. El agua va contra el
// empuje. Solo se suma al agua de la red: el empuje sobre el robot ya lo aplica Stonefish.
#pragma once

#include <optional>
#include <vector>

#include <Eigen/Core>

namespace stonefish_net::core
{

struct Jet
{
  Eigen::Vector3f origin;  // centro de la hélice, mundo
  Eigen::Vector3f dir;     // unitario, hacia donde va el agua
  float speed;             // u₀, m/s
  float radius;            // r₀, m
};

inline constexpr size_t kMaxJets = 16;  // los que caben en el bloque uniform Jets de los shaders
inline constexpr float kJetCore = 6.2f, kJetLength = 50.f;  // en diámetros del chorro contraído (2 r₀)

// El chorro de una hélice de `diameter` m centrada en `center` que empuja `thrust` N a lo largo de `thrust_axis`
// (unitario; con thrust < 0, al revés). Nada si |thrust| < 1e-6 N o diameter <= 0.
std::optional<Jet> jet_from_thrust(const Eigen::Vector3d& center, const Eigen::Vector3d& thrust_axis, double thrust,
                                   double diameter, double rho);

// Velocidad del agua que añade un chorro, o la suma de varios, en p
Eigen::Vector3f jet_velocity(const Jet& jet, const Eigen::Vector3f& p, float spread);
Eigen::Vector3f jet_velocity(const std::vector<Jet>& jets, const Eigen::Vector3f& p, float spread);

}  // namespace stonefish_net::core
