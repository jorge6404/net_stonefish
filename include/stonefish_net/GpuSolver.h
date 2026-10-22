// La red en la GPU (solver.backend: gpu): XPBD por subpasos en compute shaders (gpu/GpuXpbd), dentro del tick de
// Stonefish. Spec: docs/superpowers/specs/2026-10-20-red-gpu-design.md.
//
// begin_step (pre-tick): crea el contexto de cálculo la primera vez (ya en el hilo de la física), aplica a los
// sólidos las reacciones del tick anterior, guarda su pose y la de las boyas al inicio y sube la corriente del
// océano muestreada en una rejilla alrededor de la red.
// end_step (post-tick, con los sólidos ya movidos): une cada orinque al ojal de su boya (que cede según la masa de
// la boya, core::Anchor), ejecuta los subpasos con los sólidos en su pose interpolada, lee las posiciones, da a
// cada boya en ese momento el impulso del ancla y la lleva a donde la red dejó su ojal, y deja pendientes las
// reacciones de los sólidos. Las boyas no pueden recibir la reacción en el tick siguiente, como los sólidos: la red
// que cuelga pesa mucho más que ellas y ese acoplamiento diverge.
//
// Pinzas (grippers, core::Grasp): en begin_step agarra lo pellizcado y suelta lo que se abre, con los dedos al
// inicio del tick y la red del anterior; en end_step los nudos agarrados se atan a su dedo detrás de los anclajes de
// las boyas (sin ceder: inv_mass = 0), los dedos ignoran lo que su pinza sujeta y cada dedo recibe en el tick
// siguiente el tirón de sus nudos (SolidContacts::add_reaction). Cada agarre y suelta sale en stderr.
//
// Anclajes de un tick, en este orden: uno por boya (topo.marks.buoys) y detrás uno por nudo agarrado.
//
// Si la GPU no se puede usar, imprime el motivo y termina el proceso con 1: no vuelve a Bullet en silencio.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <SDL2/SDL.h>

#include "stonefish_net/NetSolver.h"
#include "stonefish_net/SolidContacts.h"
#include "stonefish_net/core/gpu_layout.hpp"
#include "stonefish_net/core/grasp.hpp"
#include "stonefish_net/core/net_config.hpp"
#include "stonefish_net/core/net_topology.hpp"
#include "stonefish_net/diagnostics.h"
#include "stonefish_net/gl/GlComputeContext.h"
#include "stonefish_net/gpu/GpuXpbd.h"

namespace sf
{
class SimulationManager;
class SolidEntity;
}  // namespace sf

namespace stonefish_net
{

class GpuSolver : public NetSolver
{
public:
  // Una pinza: sus dedos son cuerpos de SolidContacts (índices en bodies(), en el orden de fingers del YAML)
  struct Gripper
  {
    std::string name;  // <robot>/<dedo1>+<dedo2>…, para los mensajes
    core::Gripper spec;
  };

  // solids: los cuerpos que tocan la red (SolidContacts), que tiene que vivir más que el solver.
  // buoys: una por topo.marks.buoys, en orden; eye: el ojal en el marco de la boya.
  // window: la ventana de SDL de Stonefish (en BuildScenario, la del contexto activo del hilo principal).
  GpuSolver(const core::NetTopology& topo, const core::NetConfig& cfg, sf::SimulationManager* sm, SolidContacts& solids,
            std::vector<sf::SolidEntity*> buoys, const Eigen::Vector3d& eye, SDL_Window* window, std::string shader_dir,
            std::vector<Gripper> grippers);

  void begin_step(double dt) override;
  void end_step(double dt) override;
  const std::vector<Eigen::Vector3f>& positions() const override { return positions_; }

private:
  void start_gpu();
  void sample_current();
  void update_grasp();
  Eigen::Vector3d eye_of(size_t buoy) const;  // ojal de la boya en el mundo
  // Anclaje de cada boya en este tick; lever[k]: del centro de masa de la boya a su ojal, al final del tick
  std::vector<core::Anchor> buoy_anchors(std::vector<Eigen::Vector3d>& lever) const;
  // Tras leer de la GPU: cada boya, donde la red dejó su ojal y con su velocidad
  void move_buoys(const std::vector<core::Anchor>& anchors, const std::vector<Eigen::Vector3d>& lever, double dt);
  void sample_diagnostics(const std::vector<Eigen::Vector3d>& lever, double dt, double issue_ms, double readback_ms);

  const core::NetTopology& topo_;
  const core::NetConfig& cfg_;
  sf::SimulationManager* sm_;
  SolidContacts& solids_;
  std::vector<sf::SolidEntity*> buoys_;
  Eigen::Vector3d eye_;
  core::GpuLayout layout_;
  gl::GlComputeContext gl_;
  std::unique_ptr<gpu::GpuXpbd> xpbd_;  // en el primer begin_step, con el contexto ya activo
  std::vector<Eigen::Vector3f> positions_, anchor_dx_, anchor_offset_;  // anchor_*: por anclaje del tick
  std::vector<core::BodyReaction> reactions_;
  std::vector<Eigen::Vector3f> eye_start_;   // ojal de cada boya al inicio del tick
  std::vector<Gripper> grippers_;
  core::Grasp grasp_;
  std::unique_ptr<NetDiagnostics> diagnostics_;  // nullptr si diagnostics.enabled es false
  bool warned_grid_ = false;
};

}  // namespace stonefish_net
