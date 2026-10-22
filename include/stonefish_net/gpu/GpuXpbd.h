// El subpaso XPBD de la red en la GPU (shaders/*.comp): buffers y lanzamientos, sin Stonefish. Lo usan GpuSolver
// (dentro del simulador) y tools/gpu_check (que lo compara con core/xpbd_ref). Necesita el contexto de cálculo
// activo en el hilo que lo llama (GlComputeContext::ensure_current) desde el constructor.
//
// El contrato con los shaders (bindings, tamaños de grupo y de memoria de contacto, layout de cada struct) está en
// GpuXpbd.cpp y llega a los shaders como #define al compilarlos: no hay números que mantener a mano en los .glsl.
#pragma once

#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "stonefish_net/core/gpu_layout.hpp"
#include "stonefish_net/core/xpbd.hpp"
#include "stonefish_net/gl/GlComputeContext.h"

namespace stonefish_net::gpu
{

// El bloque uniform Params de xpbd_common.glsl (std140)
struct GpuParams
{
  float h, rho, g, cn, ct, compliance, damping, friction, floor_z;
  float pad_[3];
  float current_origin[3];
  float current_cell;
};
static_assert(sizeof(GpuParams) == 64, "Params de xpbd_common.glsl");

class GpuXpbd
{
public:
  GpuXpbd(gl::GlComputeContext& gl, const core::GpuLayout& L);
  ~GpuXpbd();
  GpuXpbd(const GpuXpbd&) = delete;
  GpuXpbd& operator=(const GpuXpbd&) = delete;

  // Velocidad del agua en una rejilla de nx × ny × nz puntos separados `cell` desde `origin` (v[ix + nx·(iy +
  // ny·iz)]); la GPU la interpola linealmente y fuera de la rejilla toma la del borde.
  void set_current_grid(const Eigen::Vector3f& origin, float cell, int nx, int ny, int nz,
                        const std::vector<Eigen::Vector3f>& v);
  void set_anchors(const std::vector<core::Anchor>& anchors);

  // Los cuerpos que tocan la red durante el tick siguiente, de n subpasos: se suben sus poses interpoladas de
  // todos los subpasos de una vez.
  void set_bodies(const std::vector<core::ContactBody>& bodies, int n);

  // El subpaso i de n, sin leer de vuelta.
  void substep(const core::XpbdParams& p, int i, int n);

  // Espera a la GPU y copia las posiciones, lo acumulado por cada anclaje y por cada cuerpo desde el último
  // read_back (que lo vuelve a cero) y lo que la red apartó cada ojal en el tick (XpbdState::anchor_offset).
  void read_back(std::vector<Eigen::Vector3f>& x, std::vector<Eigen::Vector3f>& anchor_dx,
                 std::vector<Eigen::Vector3f>& anchor_offset, std::vector<core::BodyReaction>& reactions);

  // Tiempo de GPU de los subpasos entre los dos read_back anteriores, ms.
  double gpu_ms() { return timer_.last_ms(); }

  // Contactos de más que no cupieron en el registro de reacciones del último tick (0 si cupieron todos): su
  // reacción no llega a los cuerpos.
  unsigned lost_contacts() const { return lost_contacts_; }

private:
  // Un programa con las ubicaciones de sus uniforms por lanzamiento (−1 si no los usa), buscadas una vez
  struct Program
  {
    GLuint id = 0;
    GLint n = -1, first = -1, count = -1, body_count = -1, substep = -1, substeps = -1, log_capacity = -1;
  };
  Program load(const std::string& file);
  void upload_params(const core::XpbdParams& p);  // al bloque Params, solo si han cambiado
  void bind_for_tick();                           // buffers, textura y cuántos hilos: en el primer subpaso
  void dispatch(const Program& prog, GLuint threads);
  void dispatch_colors(const Program& prog);      // un lanzamiento por color de tramos
  void clear_contact_memory();

  gl::GlComputeContext& gl_;
  GLuint nodes_ = 0, links_ = 0;
  std::vector<int> color_begin_;
  Program forces_, predict_, links_prog_, anchors_prog_, contact_nodes_, contact_links_, finish_, reduce_;

  // La red: fija, o reescrita por los shaders
  gl::Buffer x_, x_read_, v_, dx_, vnew_, node_props_, link_buf_, link_props_, adj_begin_, adj_links_;
  // Contacto: qué cuerpos ignora cada nudo, la memoria de salida de cada punto y el registro de reacciones
  gl::Buffer excluded_, node_mem_, link_mem_, node_mem_used_, link_mem_used_, log_, log_count_;
  // Crecen cuando hace falta (anchor_capacity_, body_capacity_)
  gl::Buffer anchors_, anchor_dx_, anchor_dx_read_, anchor_state_;
  gl::Buffer bodies_, reactions_, reactions_read_;
  gl::Buffer params_;
  GLuint current_ = 0;  // textura 3D de la corriente
  Eigen::Vector3f current_origin_ = Eigen::Vector3f::Zero();
  float current_cell_ = 1.f;
  std::optional<GpuParams> uploaded_;  // lo último subido a params_

  GLuint anchor_count_ = 0, anchor_capacity_ = 0, body_count_ = 0, body_capacity_ = 0, body_slots_ = 0;
  std::vector<uint32_t> excluded_now_;  // lo último subido a excluded_
  bool memory_dirty_ = false;           // la memoria de contacto tiene algo que borrar
  unsigned lost_contacts_ = 0;
  gl::GpuTimer timer_;
  bool timing_ = false;
};

}  // namespace stonefish_net::gpu
