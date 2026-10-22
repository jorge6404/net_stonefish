// Comprobaciones del solver de la red en la GPU, fuera de colcon test (necesitan una GPU con OpenGL 4.3; con NVIDIA y
// PRIME, __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia). Lo que no necesita GPU está en test/.
//
//   gpu_check gpu <net.yaml>      los shaders dan lo mismo que la referencia (core/xpbd_ref), también con cuerpos
//                                 que empujan la red, anclas que se mueven, nudos que ignoran cuerpos y el agarre
//   gpu_check bench <net.yaml>    ms por tick de la red en la GPU como en el simulador (GPU_CHECK_RATE Hz, 200)
//   gpu_check shake <net.yaml>    vibración de la red asentada, con boyas fijas y con boyas que giran
//
// Sale con 0 si todo está bien y con 1 y el primer fallo si no.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "stonefish_net/gl/GlComputeContext.h"
#include "stonefish_net/gpu/GpuXpbd.h"
#include "support/scene.hpp"

using namespace stonefish_net::test;

namespace
{
int fail(const std::string& what)
{
  std::fprintf(stderr, "FALLO: %s\n", what.c_str());
  return 1;
}

// Ventana oculta y contexto de cálculo para las órdenes de GPU
struct GpuSession
{
  SDL_Window* window = nullptr;
  std::string shader_dir;
  std::unique_ptr<stonefish_net::gl::GlComputeContext> gl;

  explicit GpuSession(const std::string& dir) : shader_dir(dir)
  {
    if (SDL_Init(SDL_INIT_VIDEO) != 0) throw std::runtime_error(SDL_GetError());
    window = SDL_CreateWindow("gpu_check", 0, 0, 16, 16, SDL_WINDOW_OPENGL | SDL_WINDOW_HIDDEN);
    if (window == nullptr) throw std::runtime_error(SDL_GetError());
    gl = std::make_unique<stonefish_net::gl::GlComputeContext>(window, shader_dir);
    gl->ensure_current();
    std::printf("GPU: %s\n", gl->vendor().c_str());
  }
  ~GpuSession()
  {
    gl.reset();
    SDL_DestroyWindow(window);
    SDL_Quit();
  }
};

// Los mismos ticks con la referencia y con los shaders; compara posiciones, lo acumulado por los anclajes y la
// reacción sobre la caja. box_speed > 0: una caja empuja la red 0,3 m a esa velocidad desde 0,35 m antes de su
// centro y luego se para 0,4 s; si no, 100 ticks sin caja.
int check_gpu(const core::NetConfig& cfg, const core::NetTopology& topo, GpuSession& gpu, bool with_anchors,
              double box_speed, const char* name)
{
  const core::GpuLayout L = core::make_gpu_layout(topo, core::color_links(topo));
  const core::XpbdParams p = params(cfg);
  const int n = cfg.solver.substeps;
  const auto water = [](const Eigen::Vector3f&) { return kCurrent; };
  const auto anchors = with_anchors ? fixed_buoys(topo) : std::vector<core::Anchor>{};
  const Eigen::Vector3d middle = net_middle(cfg);

  core::XpbdState s = core::xpbd_init(L);
  core::ContactMemory mem = core::contact_memory(L);
  std::vector<core::BodyReaction> reactions, gpu_reactions;
  std::vector<Eigen::Vector3f> ref_dx, gpu_dx, gpu_offset, gpu_x;

  stonefish_net::gpu::GpuXpbd solver(*gpu.gl, L);
  solver.set_current_grid(Eigen::Vector3f(-10.f, -10.f, -1.f), 10.f, 3, 3, 2,
                          std::vector<Eigen::Vector3f>(3 * 3 * 2, kCurrent));
  solver.set_anchors(anchors);

  // Un solo subpaso: si las cuentas son las mismas, solo difiere el redondeo de float. Con caja, la caja ya roza
  // la red (su cara frontal 2 cm más allá), para que haya contactos desde el primer subpaso. Tiene que ser un roce,
  // descentrado y algo girado en y y z: si la red queda en un plano de simetría de la caja, paralela a una cara o
  // con nudos empujados a caras opuestas (un tramo que cruza la caja tiene su punto más hundido en el plano medio),
  // la cara de salida es un empate que el redondeo decide distinto en float y en double.
  float first_substep = 0.f, first_reaction = 0.f;
  {
    std::vector<core::ContactBody> bodies;
    if (box_speed > 0.0) {
      const Eigen::Vector3d c = middle + Eigen::Vector3d(-0.28, 0.0137, 0.0071);
      const Eigen::Matrix3d rot = (Eigen::AngleAxisd(0.13, Eigen::Vector3d::UnitZ()) *
                                   Eigen::AngleAxisd(0.09, Eigen::Vector3d::UnitY())).toRotationMatrix();
      bodies.push_back(moving_box(c, c + Eigen::Vector3d(box_speed * kTick, 0.0, 0.0), rot));
    }
    core::XpbdState s1 = core::xpbd_init(L);
    core::ContactMemory m1 = core::contact_memory(L);
    std::vector<Eigen::Vector3f> dx1;
    std::vector<core::BodyReaction> r1;
    core::xpbd_substep(L, p, water, anchors, bodies, 0, n, s1, m1, dx1, r1);
    stonefish_net::gpu::GpuXpbd one(*gpu.gl, L);
    one.set_current_grid(Eigen::Vector3f(-10.f, -10.f, -1.f), 10.f, 3, 3, 2, std::vector<Eigen::Vector3f>(3 * 3 * 2, kCurrent));
    one.set_anchors(anchors);
    one.set_bodies(bodies, n);
    one.substep(p, 0, n);
    one.read_back(gpu_x, gpu_dx, gpu_offset, gpu_reactions);
    size_t worst_k = 0;
    for (size_t k = 0; k < s1.x.size(); ++k) {
      const float d = (gpu_x[k] - s1.x[k]).norm();
      if (d > first_substep) {
        first_substep = d;
        worst_k = k;
      }
    }
    if (std::getenv("GPU_CHECK_TRACE") && !bodies.empty()) {
      const Eigen::Vector3d x0(L.x0[4 * worst_k], L.x0[4 * worst_k + 1], L.x0[4 * worst_k + 2]);
      const Eigen::Vector3d local = bodies[0].pose1.inverse() * x0;
      std::printf("  nudo %zu: inicial (%.4f, %.4f, %.4f), en la caja (%.4f, %.4f, %.4f); referencia (%.4f, %.4f, %.4f), "
                  "GPU (%.4f, %.4f, %.4f)\n", worst_k, x0.x(), x0.y(), x0.z(), local.x(), local.y(), local.z(),
                  s1.x[worst_k].x(), s1.x[worst_k].y(), s1.x[worst_k].z(), gpu_x[worst_k].x(), gpu_x[worst_k].y(),
                  gpu_x[worst_k].z());
    }
    for (size_t b = 0; b < r1.size(); ++b) {
      if (r1[b].momentum.norm() < 1e-9) return fail(std::string(name) + ": la caja no toca la red en el primer subpaso");
      first_reaction = std::max(first_reaction, relative(gpu_reactions[b].momentum, r1[b].momentum));
    }
  }

  const int moving = box_speed > 0.0 ? static_cast<int>(std::lround(0.3 / box_speed / kTick)) : 0;
  const int ticks = box_speed > 0.0 ? moving + ticks_in(0.4) : ticks_in(1.0);
  Eigen::Vector3d c = middle - Eigen::Vector3d(0.35, 0.0, 0.0);
  Eigen::Vector3d ref_momentum = Eigen::Vector3d::Zero(), gpu_momentum = Eigen::Vector3d::Zero();
  // Lo acumulado por cada anclaje en toda la corrida: tick a tick, un anclaje casi quieto da un relativo sin sentido
  std::vector<Eigen::Vector3d> ref_anchor(anchors.size(), Eigen::Vector3d::Zero()), gpu_anchor = ref_anchor;
  float worst_x = 0.f, worst_dx = 0.f;
  double gpu_ms = 0.0;
  for (int t = 0; t < ticks; ++t) {
    std::vector<core::ContactBody> bodies;
    if (box_speed > 0.0) {
      const Eigen::Vector3d c1 = c + Eigen::Vector3d(t < moving ? box_speed * kTick : 0.0, 0.0, 0.0);
      bodies.push_back(moving_box(c, c1));
      c = c1;
    }
    ref_dx.assign(anchors.size(), Eigen::Vector3f::Zero());
    reactions.assign(bodies.size(), core::BodyReaction{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()});
    for (int i = 0; i < n; ++i) core::xpbd_substep(L, p, water, anchors, bodies, i, n, s, mem, ref_dx, reactions);
    solver.set_bodies(bodies, n);
    for (int i = 0; i < n; ++i) solver.substep(p, i, n);  // seguidos, como en el simulador
    solver.read_back(gpu_x, gpu_dx, gpu_offset, gpu_reactions);
    gpu_ms += solver.gpu_ms();
    if (const GLenum err = glGetError(); err != GL_NO_ERROR) {
      return fail(std::string(name) + ": error de OpenGL " + std::to_string(err) + " en el tick " + std::to_string(t));
    }
    for (size_t k = 0; k < s.x.size(); ++k) worst_x = std::max(worst_x, (gpu_x[k] - s.x[k]).norm());
    if (std::getenv("GPU_CHECK_TRACE") && (t < 3 || t % 10 == 9)) std::printf("  tick %3d: %.2e m\n", t + 1, worst_x);
    for (size_t j = 0; j < anchors.size(); ++j) {
      ref_anchor[j] += ref_dx[j].cast<double>();
      gpu_anchor[j] += gpu_dx[j].cast<double>();
    }
    if (!bodies.empty()) {
      ref_momentum += reactions[0].momentum;
      gpu_momentum += gpu_reactions[0].momentum;
    }
  }
  for (size_t j = 0; j < anchors.size(); ++j) worst_dx = std::max(worst_dx, relative(gpu_anchor[j], ref_anchor[j]));
  const float worst_reaction = box_speed > 0.0 ? relative(gpu_momentum, ref_momentum) : 0.f;
  // El tiempo es orientativo: mientras la CPU calcula la referencia la GPU baja de frecuencia
  std::printf("%s: %zu nudos, %d colores, diferencia en 1 subpaso %.2e m (reacción %.3f %%), en %d ticks %.2e m, "
              "anclajes %.2f %%, reacción sobre la caja %.2f %%, GPU %.3f ms/tick (orientativo)\n",
              name, s.x.size(), L.colors(), first_substep, 100.f * first_reaction, ticks, worst_x, 100.f * worst_dx,
              100.f * worst_reaction, gpu_ms / ticks);
  // Las cuentas son las mismas si en un subpaso solo difiere el redondeo de float. Con contacto de tramos, además, el
  // punto más hundido sale de una sección áurea de 20 pasos (un intervalo de 7e-5 del tramo) que en float y en double
  // no acaba en el mismo punto: hasta 0,1 mm. Después, la red amplifica esas diferencias (sobre todo cuando un tramo
  // pasa de flojo a tenso): en los ticks solo se pide que no se desboque.
  const float substep_tolerance = box_speed > 0.0 ? 1e-4f : 1e-6f;
  if (!(first_substep <= substep_tolerance)) return fail(std::string(name) + ": en un subpaso la GPU no hace las mismas cuentas");
  if (!(first_reaction <= 1e-3f)) return fail(std::string(name) + ": en un subpaso la reacción de la GPU no coincide");
  if (!(worst_x <= 5e-3f)) return fail(std::string(name) + ": la GPU se separa de la referencia");
  if (!(worst_dx <= 0.05f)) return fail(std::string(name) + ": los anclajes de la GPU no coinciden");
  if (!(worst_reaction <= 0.05f)) return fail(std::string(name) + ": la reacción de la GPU sobre la caja no coincide");
  return 0;
}
// El ancla en movimiento de Xpbd.MovingAnchorIsSmooth (test_xpbd), en la GPU y en la referencia: mismo resultado
int check_gpu_moving_anchor(const core::NetConfig& cfg, GpuSession& gpu)
{
  const int n = cfg.solver.substeps;
  const core::GpuLayout L = hanging_chain(5);
  const core::XpbdParams p = dry(cfg);
  const Eigen::Vector3f u(0.1f, 0.f, 0.05f);
  core::XpbdState s = core::xpbd_init(L);
  core::ContactMemory mem = core::contact_memory(L);
  std::vector<core::BodyReaction> reactions, gpu_reactions;
  std::vector<Eigen::Vector3f> ref_moved, gpu_moved, gpu_offset, gpu_x;
  const auto water = [](const Eigen::Vector3f&) { return Eigen::Vector3f::Zero(); };
  stonefish_net::gpu::GpuXpbd solver(*gpu.gl, L);
  Eigen::Vector3f eye = Eigen::Vector3f::Zero();
  float worst_x = 0.f, worst_moved = 0.f, worst_offset = 0.f;
  for (int t = 0; t < ticks_in(0.1); ++t) {
    const Eigen::Vector3f next = eye + static_cast<float>(kTick) * u;
    // Boya de 50 g: el ancla cede ante la cadena (100 g) y lo nota en la posición
    core::Anchor a{0, eye, next};
    a.inv_mass = core::point_inv_mass(0.05, Eigen::Matrix3d::Zero(), Eigen::Vector3d::Zero());
    const std::vector<core::Anchor> anchors{a};
    ref_moved.assign(1, Eigen::Vector3f::Zero());
    for (int i = 0; i < n; ++i) core::xpbd_substep(L, p, water, anchors, {}, i, n, s, mem, ref_moved, reactions);
    solver.set_anchors(anchors);
    solver.set_bodies({}, n);
    for (int i = 0; i < n; ++i) solver.substep(p, i, n);
    solver.read_back(gpu_x, gpu_moved, gpu_offset, gpu_reactions);
    for (size_t k = 0; k < s.x.size(); ++k) worst_x = std::max(worst_x, (gpu_x[k] - s.x[k]).norm());
    worst_moved = std::max(worst_moved, relative(gpu_moved[0].cast<double>(), ref_moved[0].cast<double>()));
    worst_offset = std::max(worst_offset, (gpu_offset[0] - s.anchor_offset[0]).norm());
    eye = next;
  }
  std::printf("gpu_moving_anchor: diferencia máx de posición %.2e m, de lo movido por el ancla %.3f %%, del "
              "apartado del ojal %.2e m (%.2e m en la referencia)\n", worst_x, 100.f * worst_moved, worst_offset,
              s.anchor_offset[0].norm());
  // Lo movido por el ancla en un tick es una suma en float de n aportaciones pequeñas: 1 % de margen
  if (!(worst_x <= 1e-5f && worst_moved <= 1e-2f && worst_offset <= 1e-6f)) {
    return fail("gpu_moving_anchor: la GPU no sigue al ancla igual");
  }
  return 0;
}

// Al final del escenario GpuSolver se destruye en otro hilo que el de su contexto, quizá con el contexto de dibujo
// de Stonefish activo: no puede borrar nada en ese contexto (los nombres de buffer de uno y otro coinciden)
int check_destructor_other_context(GpuSession& gpu, const core::GpuLayout& L)
{
  SDL_GLContext stonefish = SDL_GL_CreateContext(gpu.window);  // queda activo en este hilo, como el de Stonefish
  GLuint theirs[4] = {0, 0, 0, 0};
  glGenBuffers(4, theirs);
  for (GLuint b : theirs) {
    glBindBuffer(GL_ARRAY_BUFFER, b);
    glBufferData(GL_ARRAY_BUFFER, 16, nullptr, GL_STATIC_DRAW);
  }
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  std::unique_ptr<stonefish_net::gl::GlComputeContext> ctx;
  std::unique_ptr<stonefish_net::gpu::GpuXpbd> solver;
  std::string error;
  std::thread physics([&] {
    try {
      ctx = std::make_unique<stonefish_net::gl::GlComputeContext>(gpu.window, gpu.shader_dir);
      ctx->ensure_current();
      solver = std::make_unique<stonefish_net::gpu::GpuXpbd>(*ctx, L);
      glFinish();
    } catch (const std::exception& e) {
      error = e.what();
    }
  });
  physics.join();
  if (!error.empty()) return fail("destructor: " + error);

  SDL_GL_MakeCurrent(gpu.window, stonefish);
  solver.reset();
  ctx.reset();
  int alive = 0;
  for (GLuint b : theirs) alive += glIsBuffer(b) == GL_TRUE;
  glDeleteBuffers(4, theirs);
  SDL_GL_MakeCurrent(gpu.window, nullptr);
  SDL_GL_DeleteContext(stonefish);
  std::printf("destructor: %d de 4 buffers del otro contexto siguen vivos\n", alive);
  if (alive != 4) return fail("destructor: GpuXpbd borra buffers del contexto activo, que no es el suyo");
  return 0;
}

// El subpaso de test_xpbd Xpbd.ExclusionMask en la GPU: mismas posiciones y reacciones
int check_gpu_exclusion(const core::NetConfig& cfg, const core::NetTopology& topo, GpuSession& gpu)
{
  const core::GpuLayout L = core::make_gpu_layout(topo, core::color_links(topo));
  const ExclusionCase e = exclusion_case(cfg, L);
  const core::XpbdParams p = params(cfg);
  const int n = cfg.solver.substeps;
  core::XpbdState s = core::xpbd_init(L);
  core::ContactMemory mem = core::contact_memory(L);
  std::vector<Eigen::Vector3f> anchor_dx, gpu_x, gpu_dx, gpu_offset;
  std::vector<core::BodyReaction> r, gpu_r;
  const auto water = [](const Eigen::Vector3f&) { return kCurrent; };
  core::xpbd_substep(L, p, water, {}, e.bodies, 0, n, s, mem, anchor_dx, r);
  stonefish_net::gpu::GpuXpbd solver(*gpu.gl, L);
  solver.set_current_grid(Eigen::Vector3f(-10.f, -10.f, -1.f), 10.f, 3, 3, 2,
                          std::vector<Eigen::Vector3f>(3 * 3 * 2, kCurrent));
  solver.set_anchors({});
  solver.set_bodies(e.bodies, n);
  solver.substep(p, 0, n);
  solver.read_back(gpu_x, gpu_dx, gpu_offset, gpu_r);
  if (const GLenum err = glGetError(); err != GL_NO_ERROR) {
    return fail("gpu_exclusion: error de OpenGL " + std::to_string(err));
  }
  float worst_x = 0.f, worst_r = 0.f;
  size_t worst_k = 0;
  for (size_t k = 0; k < s.x.size(); ++k) {
    if ((gpu_x[k] - s.x[k]).norm() > worst_x) {
      worst_x = (gpu_x[k] - s.x[k]).norm();
      worst_k = k;
    }
  }
  for (size_t b = 0; b < r.size(); ++b) worst_r = std::max(worst_r, relative(gpu_r[b].momentum, r[b].momentum));
  const auto ek = static_cast<size_t>(e.node);
  const float moved = (s.x[ek] - Eigen::Vector3f(L.x0[4 * ek], L.x0[4 * ek + 1], L.x0[4 * ek + 2])).norm();
  std::printf("gpu_exclusion: diferencia %.2e m (nudo %zu; el %d se movió %.2e m), reacción %.3f %%\n", worst_x,
              worst_k, e.node, moved, 100.f * worst_r);
  // El nudo sale 3 mm de la caja que no excluye: la geometría del contacto en float (GPU) y en double (referencia), a
  // unos metros del origen, difiere en micras. Si excluyera mal, se movería milímetros de más o de menos.
  if (!(worst_x <= 1e-5f)) return fail("gpu_exclusion: la GPU no excluye igual que la referencia");
  if (!(worst_r <= 1e-3f)) return fail("gpu_exclusion: la reacción de la GPU no coincide");
  return 0;
}

// Un tick en la GPU como NetTick; gl_error recibe el primer error de OpenGL
NetTick gpu_tick(const core::NetConfig& cfg, const core::GpuLayout& L, GpuSession& gpu, GLenum& gl_error)
{
  struct Gpu
  {
    Gpu(GpuSession& session, const core::GpuLayout& layout) : solver(*session.gl, layout) {}
    stonefish_net::gpu::GpuXpbd solver;
    std::vector<Eigen::Vector3f> offset;
    std::vector<core::BodyReaction> reactions;
  };
  auto g = std::make_shared<Gpu>(gpu, L);
  g->solver.set_current_grid(Eigen::Vector3f(-10.f, -10.f, -1.f), 10.f, 3, 3, 2,
                             std::vector<Eigen::Vector3f>(3 * 3 * 2, kCurrent));
  const core::XpbdParams p = params(cfg);
  const int n = cfg.solver.substeps;
  return [g, p, n, &gl_error](const std::vector<core::Anchor>& anchors, const std::vector<core::ContactBody>& bodies,
                              std::vector<Eigen::Vector3f>& x, std::vector<Eigen::Vector3f>& anchor_dx) {
    g->solver.set_anchors(anchors);
    g->solver.set_bodies(bodies, n);
    for (int i = 0; i < n; ++i) g->solver.substep(p, i, n);
    g->solver.read_back(x, anchor_dx, g->offset, g->reactions);
    if (const GLenum err = glGetError(); err != GL_NO_ERROR && gl_error == GL_NO_ERROR) gl_error = err;
  };
}

// La prueba Grasp.PullAndRelease (test_grasp) con la referencia y con la GPU a la vez, cada una con su propio core::Grasp: agarran los mismos
// nudos, no se separan y lo acumulado por los anclajes del agarre coincide (el número de anclajes cambia al agarrar
// y al soltar, y GpuXpbd rehace sus buffers)
int check_gpu_grip(const core::NetConfig& cfg, const core::NetTopology& topo, GpuSession& gpu)
{
  const core::GpuLayout L = core::make_gpu_layout(topo, core::color_links(topo));
  GLenum gl_error = GL_NO_ERROR;
  GripScene ref(topo, ref_tick(cfg, L)), dev(topo, gpu_tick(cfg, L, gpu, gl_error));
  ref.settle(5.0);
  dev.settle(5.0);
  float worst_x = 0.f;
  const auto compare = [&] {
    for (size_t k = 0; k < ref.x.size(); ++k) worst_x = std::max(worst_x, (dev.x[k] - ref.x[k]).norm());
  };
  compare();
  const float settled = worst_x;
  ref.pinch = dev.pinch = ref.nearest_node(net_middle(cfg));
  for (int t = 0; t < GripScene::ticks(); ++t) {
    ref.step(t);
    dev.step(t);
    if (gl_error != GL_NO_ERROR) return fail("gpu_grip: error de OpenGL " + std::to_string(gl_error) + " en el tick " + std::to_string(t));
    compare();
  }
  const float dx = relative(dev.grip_dx, ref.grip_dx);
  std::printf("gpu_grip: agarran %zu y %zu nudos, diferencia asentada %.2e m, en toda la prueba %.2e m, anclajes del "
              "agarre %.2f %%, quedan %zu y %zu agarrados\n", ref.first_held.size(), dev.first_held.size(), settled,
              worst_x, 100.f * dx, ref.grasp.held(0).size(), dev.grasp.held(0).size());
  if (ref.first_held.empty()) return fail("gpu_grip: la referencia no agarra nada");
  if (dev.first_held != ref.first_held) return fail("gpu_grip: la GPU no agarra los mismos nudos que la referencia");
  if (!(worst_x <= 5e-3f)) return fail("gpu_grip: la GPU se separa de la referencia");
  if (!(dx <= 0.05f)) return fail("gpu_grip: lo acumulado por los anclajes del agarre no coincide");
  if (!dev.grasp.held(0).empty()) return fail("gpu_grip: la GPU no lo suelta todo");
  return 0;
}

// Coste de un tick como en el simulador: la red con sus boyas ancladas y 7 cuerpos, a simulation_rate Hz
// (GPU_CHECK_RATE, 200 por defecto) con los subpasos del YAML; primero con los cuerpos lejos de la red y luego con
// uno que la empuja 0,3 m a 0,5 m/s. Mide lo que la CPU tarda en mandar los subpasos, lo que espera a la GPU y lee,
// y el tiempo de GPU. Sin el dibujo de Stonefish compitiendo por la GPU: en el simulador, más.
int bench(const core::NetConfig& cfg, const core::NetTopology& topo, GpuSession& gpu)
{
  const double rate = std::getenv("GPU_CHECK_RATE") ? std::atof(std::getenv("GPU_CHECK_RATE")) : 200.0;
  const double dt = 1.0 / rate;
  const int n = cfg.solver.substeps;
  const core::GpuLayout L = core::make_gpu_layout(topo, core::color_links(topo));
  core::XpbdParams p = params(cfg);
  p.h = static_cast<float>(dt / n);
  const Eigen::Vector3d middle = net_middle(cfg);
  for (const bool push : {false, true}) {
    stonefish_net::gpu::GpuXpbd solver(*gpu.gl, L);
    solver.set_current_grid(Eigen::Vector3f(-10.f, -10.f, -1.f), 10.f, 3, 3, 2,
                            std::vector<Eigen::Vector3f>(3 * 3 * 2, kCurrent));
    solver.set_anchors(fixed_buoys(topo));
    std::vector<Eigen::Vector3d> centers;
    for (int b = 0; b < 7; ++b) centers.push_back(middle + Eigen::Vector3d(push && b == 0 ? -0.35 : -2.0, 0.6 * b, 0.0));
    const int moving = static_cast<int>(std::lround(0.3 / 0.5 / dt));
    std::vector<Eigen::Vector3f> x, dx, offset;
    std::vector<core::BodyReaction> reactions;
    const int warmup = 50, ticks = 400;
    double issue_ms = 0.0, wait_ms = 0.0, gpu_ms = 0.0, worst_ms = 0.0;
    for (int t = 0; t < warmup + ticks; ++t) {
      std::vector<core::ContactBody> bodies;
      for (size_t b = 0; b < centers.size(); ++b) {
        const Eigen::Vector3d c1 = centers[b] + Eigen::Vector3d(push && b == 0 && t < moving ? 0.5 * dt : 0.0, 0.0, 0.0);
        bodies.push_back(moving_box(centers[b], c1));
        centers[b] = c1;
      }
      const auto t0 = std::chrono::steady_clock::now();
      solver.set_bodies(bodies, n);
      for (int i = 0; i < n; ++i) solver.substep(p, i, n);
      const auto t1 = std::chrono::steady_clock::now();
      solver.read_back(x, dx, offset, reactions);
      const auto t2 = std::chrono::steady_clock::now();
      if (t < warmup) continue;
      const double a = std::chrono::duration<double, std::milli>(t1 - t0).count();
      const double b = std::chrono::duration<double, std::milli>(t2 - t1).count();
      issue_ms += a;
      wait_ms += b;
      worst_ms = std::max(worst_ms, a + b);
      gpu_ms += solver.gpu_ms();
    }
    std::printf("bench %s: %zu nudos, %d colores, %d subpasos a %.0f Hz: CPU mandando %.3f ms/tick, espera y lectura "
                "%.3f ms/tick, total %.3f ms/tick (peor %.3f) de %.3f ms; GPU %.3f ms/tick\n",
                push ? "empujando" : "lejos", L.node_count(), L.colors(), n, rate, issue_ms / ticks, wait_ms / ticks,
                (issue_ms + wait_ms) / ticks, worst_ms, 1e3 * dt, gpu_ms / ticks);
  }
  return 0;
}

// Boyas como sólidos rígidos que giran (como en Bullet: masa 0,5 kg, inercia del asset, ojal 0,146 m por debajo del
// centro de gravedad, empuje según lo hundidas, arrastre lineal y angular), acopladas a la red en la GPU como en
// GpuSolver::end_step. `rigid_offset`: el apartado del ojal se reparte entre traslación y giro según la masa y la
// inercia (si no, toda la boya se traslada). Devuelve en el último segundo cuántas veces por tick invierte la
// velocidad el ojal y el temblor rms de los nudos.
struct BuoyShake
{
  double flips_per_tick, eye_speed, jitter_mm, w_max;
};
BuoyShake shake_buoys(const core::NetConfig& cfg, const core::NetTopology& topo, GpuSession& gpu, double dt,
                      bool rigid_offset, double seconds)
{
  const int n = cfg.solver.substeps;
  const core::GpuLayout L = core::make_gpu_layout(topo, core::color_links(topo));
  core::XpbdParams p = params(cfg);
  p.h = static_cast<float>(dt / n);
  p.damping = core::substep_damping(cfg.solver.damping, n);
  stonefish_net::gpu::GpuXpbd solver(*gpu.gl, L);
  solver.set_current_grid(Eigen::Vector3f(-10.f, -10.f, -1.f), 10.f, 3, 3, 2,
                          std::vector<Eigen::Vector3f>(3 * 3 * 2, kCurrent));
  solver.set_bodies({}, n);
  const double mass = cfg.buoys.mass, volume = 6.2738e-3, half_height = 0.151, radius = 0.113, rho = cfg.hydro.water_density;
  const double g = 9.81, area = 3.14159265358979 * radius * radius;
  const Eigen::Matrix3d inertia = Eigen::Vector3d(0.002, 0.003, 0.003).asDiagonal();
  const Eigen::Vector3d eye_local(0.0, 0.0, 0.146);  // abajo (NED)
  const size_t count = topo.marks.buoys.size();
  std::vector<Eigen::Vector3d> cg(count), v(count, Eigen::Vector3d::Zero()), w(count, Eigen::Vector3d::Zero());
  std::vector<Eigen::Matrix3d> R(count, Eigen::Matrix3d::Identity());
  for (size_t k = 0; k < count; ++k) cg[k] = topo.nodes[static_cast<size_t>(topo.marks.buoys[k].node)].position - eye_local;
  std::vector<Eigen::Vector3f> x, prev, prev2, dx, offset;
  std::vector<core::BodyReaction> reactions;
  std::vector<Eigen::Vector3d> eye_v_prev(count, Eigen::Vector3d::Zero()), held_f(count), held_torque(count);
  const int prescaler = std::max(1, static_cast<int>(std::lround(1.0 / dt / 50.0)));
  const int ticks = static_cast<int>(std::lround(seconds / dt)), last = static_cast<int>(std::lround(1.0 / dt));
  long flips = 0;
  double eye_speed = 0.0, jitter2 = 0.0, w_max = 0.0;
  long samples = 0;
  for (int t = 0; t < ticks; ++t) {
    std::vector<core::Anchor> anchors;
    std::vector<Eigen::Vector3d> start(count), lever(count);
    for (size_t k = 0; k < count; ++k) {
      start[k] = cg[k] + R[k] * eye_local;
      // Bullet: fuerzas sin la red, Euler semiimplícito. Stonefish recalcula empuje y arrastre cada
      // round(rate/50) ticks y entre medias repite los últimos
      if (t % prescaler == 0) {
        const double wet = std::clamp(cg[k].z() / (2.0 * half_height) + 0.5, 0.0, 1.0);
        held_f[k] = -0.5 * rho * area * v[k].norm() * v[k] + Eigen::Vector3d(0.0, 0.0, -rho * volume * wet * g);
        held_torque[k] = -0.5 * rho * std::pow(radius, 5) * w[k].norm() * w[k];
      }
      const Eigen::Vector3d f = held_f[k] + Eigen::Vector3d(0.0, 0.0, mass * g), torque = held_torque[k];
      const Eigen::Matrix3d inv_I = R[k] * inertia.inverse() * R[k].transpose();
      v[k] += dt * f / mass;
      w[k] += dt * inv_I * torque;
      cg[k] += dt * v[k];
      if (w[k].norm() > 0.0) R[k] = Eigen::AngleAxisd(w[k].norm() * dt, w[k].normalized()).toRotationMatrix() * R[k];
      lever[k] = R[k] * eye_local;
      core::Anchor a{topo.marks.buoys[k].node, start[k].cast<float>(), (cg[k] + lever[k]).cast<float>()};
      a.inv_mass = core::point_inv_mass(mass, R[k] * inertia.inverse() * R[k].transpose(), lever[k]);
      anchors.push_back(a);
    }
    solver.set_anchors(anchors);
    for (int i = 0; i < n; ++i) solver.substep(p, i, n);
    prev2 = prev;
    prev = x;
    solver.read_back(x, dx, offset, reactions);
    for (size_t k = 0; k < count; ++k) {
      const Eigen::Matrix3d K = anchors[k].inv_mass.cast<double>();
      const Eigen::Matrix3d inv_I = R[k] * inertia.inverse() * R[k].transpose();
      const Eigen::Vector3d off = offset[k].cast<double>();
      const Eigen::Vector3d end = anchors[k].to.cast<double>() + off;
      const Eigen::Vector3d dv = (end - start[k]) / dt - (v[k] + w[k].cross(lever[k]));
      const Eigen::Vector3d J = core::eye_impulse(K, dv);
      v[k] += J / mass;
      w[k] += inv_I * lever[k].cross(J);
      if (rigid_offset) {
        const Eigen::Vector3d lambda = core::eye_impulse(K, off);
        cg[k] += lambda / mass;
        const Eigen::Vector3d turn = inv_I * lever[k].cross(lambda);
        if (turn.norm() > 0.0) R[k] = Eigen::AngleAxisd(turn.norm(), turn.normalized()).toRotationMatrix() * R[k];
      } else {
        cg[k] += off;
      }
      const Eigen::Vector3d eye_v = v[k] + w[k].cross(R[k] * eye_local);
      if (t >= ticks - last) {
        if (eye_v.norm() > 0.005 && eye_v.dot(eye_v_prev[k]) < 0.0) ++flips;
        eye_speed = std::max(eye_speed, eye_v.norm());
        w_max = std::max(w_max, w[k].norm());
      }
      eye_v_prev[k] = eye_v;
    }
    if (t >= ticks - last && !prev2.empty()) {
      for (size_t k = 0; k < x.size(); ++k) {
        const double j = (x[k] - 2.f * prev[k] + prev2[k]).norm();
        jitter2 += j * j;
        ++samples;
      }
    }
  }
  return {static_cast<double>(flips) / (count * last), eye_speed, 1e3 * std::sqrt(jitter2 / std::max(samples, 1L)), w_max};
}

// Vibración de la red asentada en la GPU, con la corriente del tanque y las boyas quietas, a GPU_CHECK_RATE Hz (200
// por defecto): tras GPU_CHECK_SETTLE s (8), durante 1 s, velocidad media de los nudos (de las posiciones de cada
// tick, como el diagnóstico), qué parte de los nudos que se mueven invierte su velocidad de un tick al siguiente y la
// segunda diferencia de las posiciones (lo que tiembla de un tick a otro).
int shake(const core::NetConfig& cfg, const core::NetTopology& topo, GpuSession& gpu)
{
  const double rate = std::getenv("GPU_CHECK_RATE") ? std::atof(std::getenv("GPU_CHECK_RATE")) : 200.0;
  const double settle = std::getenv("GPU_CHECK_SETTLE") ? std::atof(std::getenv("GPU_CHECK_SETTLE")) : 8.0;
  const double dt = 1.0 / rate;
  const int n = cfg.solver.substeps;
  const core::GpuLayout L = core::make_gpu_layout(topo, core::color_links(topo));
  core::XpbdParams p = params(cfg);
  p.h = static_cast<float>(dt / n);
  p.damping = core::substep_damping(cfg.solver.damping, n);
  stonefish_net::gpu::GpuXpbd solver(*gpu.gl, L);
  solver.set_current_grid(Eigen::Vector3f(-10.f, -10.f, -1.f), 10.f, 3, 3, 2,
                          std::vector<Eigen::Vector3f>(3 * 3 * 2, kCurrent));
  solver.set_anchors(fixed_buoys(topo));
  solver.set_bodies({}, n);
  std::vector<Eigen::Vector3f> x, prev, prev2, dx, offset;
  std::vector<core::BodyReaction> reactions;
  const int settle_ticks = static_cast<int>(std::lround(settle * rate)), measure = static_cast<int>(std::lround(rate));
  double vsum = 0.0, jitter2 = 0.0, vmax = 0.0, jmax = 0.0;
  long moving = 0, flips = 0, samples = 0;
  size_t worst = 0;
  for (int t = 0; t < settle_ticks + measure; ++t) {
    for (int i = 0; i < n; ++i) solver.substep(p, i, n);
    prev2 = prev;
    prev = x;
    solver.read_back(x, dx, offset, reactions);
    if (t < settle_ticks || prev2.empty()) continue;
    for (size_t k = 0; k < x.size(); ++k) {
      const Eigen::Vector3f v1 = (x[k] - prev[k]) / static_cast<float>(dt), v0 = (prev[k] - prev2[k]) / static_cast<float>(dt);
      const double j = (x[k] - 2.f * prev[k] + prev2[k]).norm();
      vsum += v1.norm();
      vmax = std::max(vmax, static_cast<double>(v1.norm()));
      jitter2 += j * j;
      if (j > jmax) {
        jmax = j;
        worst = k;
      }
      ++samples;
      if (v1.norm() > 0.005f) {
        ++moving;
        if (v1.dot(v0) < 0.f) ++flips;
      }
    }
  }
  const char* kinds[] = {"red", "orinque", "fondeo"};
  std::printf("shake: %zu nudos, %d subpasos a %.0f Hz (h %.3f ms): |v| media %.4f m/s, máx %.4f; se mueven %.1f %% de "
              "los nudos, de ellos invierten la velocidad cada tick %.1f %%; temblor (2.ª diferencia) rms %.3f mm, máx "
              "%.3f mm (nudo %zu, %s, %d tramos, z %.2f)\n",
              x.size(), n, rate, 1e3 * dt / n, vsum / samples, vmax, 100.0 * moving / samples,
              moving ? 100.0 * flips / moving : 0.0, 1e3 * std::sqrt(jitter2 / samples), 1e3 * jmax, worst,
              kinds[static_cast<int>(topo.nodes[worst].kind)], L.adj_begin[worst + 1] - L.adj_begin[worst], x[worst].z());
  for (const bool rigid : {false, true}) {
    const BuoyShake b = shake_buoys(cfg, topo, gpu, dt, rigid, settle + 1.0);
    std::printf("shake con boyas que giran (%s): el ojal invierte la velocidad en %.2f de cada tick, |v| máx del ojal %.3f "
                "m/s, |w| máx %.2f rad/s; temblor de la red rms %.3f mm\n",
                rigid ? "apartado con traslación y giro" : "apartado solo trasladando", b.flips_per_tick, b.eye_speed,
                b.w_max, b.jitter_mm);
  }
  return 0;
}
}  // namespace

int main(int argc, char** argv)
{
  if (argc < 3) {
    std::fprintf(stderr, "Uso: gpu_check gpu|bench|shake <net.yaml> [carpeta de shaders]\n");
    return 2;
  }
  const std::string cmd = argv[1];
  try {
    const core::NetConfig cfg = core::load_net_config(argv[2]);
    const core::NetTopology topo = core::build_topology(cfg);
    GpuSession gpu(argc > 3 ? argv[3] : "src/stonefish_net/shaders");
    if (cmd == "gpu") {
      int rc = check_gpu(cfg, topo, gpu, true, 0.0, "gpu");
      rc |= check_gpu(cfg, topo, gpu, false, 0.0, "no_bodies");
      rc |= check_gpu(cfg, topo, gpu, true, 0.5, "push_box");
      rc |= check_gpu(cfg, topo, gpu, true, 2.0, "fast_box");
      rc |= check_gpu_moving_anchor(cfg, gpu);
      rc |= check_gpu_exclusion(cfg, topo, gpu);
      rc |= check_gpu_grip(cfg, topo, gpu);
      rc |= check_destructor_other_context(gpu, hanging_chain(5));
      return rc;
    }
    if (cmd == "shake") {
      return shake(cfg, topo, gpu);
    }
    if (cmd == "bench") {
      return bench(cfg, topo, gpu);
    }
  } catch (const std::exception& e) {
    return fail(e.what());
  }
  std::fprintf(stderr, "orden desconocida '%s' (colors y ref son ahora colcon test: test_topology, test_xpbd, ...)\n",
               cmd.c_str());
  return 2;
}
