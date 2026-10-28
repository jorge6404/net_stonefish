#include "stonefish_net/gpu/GpuXpbd.h"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <iterator>
#include <utility>

#include <Eigen/Geometry>

#include "stonefish_net/core/hydro.hpp"

namespace stonefish_net::gpu
{
namespace
{
// --- Contrato con los shaders: llega a ellos como #define (shader_defines) ---

constexpr GLuint kGroup = 64;              // local_size_x de los shaders
constexpr GLuint kLogCapacity = 1u << 20;  // contactos por tick en el registro de reacciones (32 MB)

// Bindings de los SSBO (std430) y del bloque uniform Params (std140)
enum Binding : GLuint
{
  kPositions, kVelocities, kDisplacements, kNewVelocities, kNodeProps, kLinks, kLinkProps, kAdjacencyBegin,
  kAdjacencyLinks, kAnchors, kAnchorMoved, kBodies, kExcluded, kNodeMemory, kLinkMemory, kReactionLog, kReactionCount,
  kReactions, kNodeMemoryUsed, kLinkMemoryUsed, kAnchorState
};
constexpr GLuint kParamsBinding = 0, kJetsBinding = 1;  // bloques uniform (std140)

std::string shader_defines()
{
  const std::pair<const char*, unsigned long> defines[] = {
    {"GROUP_SIZE", kGroup},
    {"MAX_BODIES", core::kMaxContactBodies},
    {"MAX_EXCLUDING_BODIES", core::kMaxExcludingBodies},
    {"B_PARAMS", kParamsBinding},
    {"B_JETS", kJetsBinding},
    {"MAX_JETS", core::kMaxJets},
    {"B_POSITIONS", kPositions},
    {"B_VELOCITIES", kVelocities},
    {"B_DISPLACEMENTS", kDisplacements},
    {"B_NEW_VELOCITIES", kNewVelocities},
    {"B_NODE_PROPS", kNodeProps},
    {"B_LINKS", kLinks},
    {"B_LINK_PROPS", kLinkProps},
    {"B_ADJACENCY_BEGIN", kAdjacencyBegin},
    {"B_ADJACENCY_LINKS", kAdjacencyLinks},
    {"B_ANCHORS", kAnchors},
    {"B_ANCHOR_MOVED", kAnchorMoved},
    {"B_BODIES", kBodies},
    {"B_EXCLUDED", kExcluded},
    {"B_NODE_MEMORY", kNodeMemory},
    {"B_LINK_MEMORY", kLinkMemory},
    {"B_REACTION_LOG", kReactionLog},
    {"B_REACTION_COUNT", kReactionCount},
    {"B_REACTIONS", kReactions},
    {"B_NODE_MEMORY_USED", kNodeMemoryUsed},
    {"B_LINK_MEMORY_USED", kLinkMemoryUsed},
    {"B_ANCHOR_STATE", kAnchorState},
  };
  std::string out;
  for (const auto& [name, value] : defines) out += "#define " + std::string(name) + " " + std::to_string(value) + "\n";
  // Los float, con su punto ("0.470000")
  out += "#define KNOT_CD " + std::to_string(core::kKnotCd) + "\n";
  out += "#define JET_CORE " + std::to_string(core::kJetCore) + "\n";
  out += "#define JET_LENGTH " + std::to_string(core::kJetLength) + "\n";
  return out;
}

// Layouts std430 de los structs de los shaders
struct GpuVec4
{
  float v[4];
};
struct GpuAnchor  // AnchorData: el ojal al inicio y al final del tick, el nudo y las columnas de inv_mass
{
  float from[4], to[4];
  int32_t node[4];
  float inv_mass[3][4];
};
struct GpuBody  // BodyData de un cuerpo en un subpaso
{
  float to_local[16], to_world[16];  // por columnas, como Eigen y GLSL
  float half_rounding[4], offset_friction[4], cg[4], v_cg[4], w[4];
};
struct GpuMoved  // LogEntry y cada reacción: Σ m·Δx (w = cuerpo, en el registro) y Σ r × m·Δx
{
  float momentum[4], angular[4];
};
struct GpuAnchorState  // por anclaje, dentro del tick: lo que la red apartó el ojal y su velocidad
{
  float offset[4], vel[4];
};
static_assert(sizeof(GpuVec4) == 16 && sizeof(GpuAnchor) == 96 && sizeof(GpuBody) == 208 && sizeof(GpuMoved) == 32 &&
              sizeof(GpuAnchorState) == 32, "layouts std430 de xpbd_common.glsl y contact_common.glsl");

// --- Ayudas ---

template <typename T>
GLsizeiptr bytes_of(const std::vector<T>& v)
{
  return static_cast<GLsizeiptr>(v.size() * sizeof(T));
}

template <typename T>
gl::Buffer buffer_of(const std::vector<T>& v)
{
  return gl::Buffer(bytes_of(v), v.empty() ? nullptr : v.data());
}

// Si `needed` no cabe en `capacity`, la amplía (al menos a 1) y devuelve true: hay que rehacer sus buffers
bool grow(GLuint& capacity, GLuint needed)
{
  if (capacity > 0 && needed <= capacity) return false;
  capacity = std::max<GLuint>(needed, 1);
  return true;
}

void fill(float (&out)[4], const Eigen::Vector3f& v, float w = 0.f) { out[0] = v.x(), out[1] = v.y(), out[2] = v.z(), out[3] = w; }
void fill(float (&out)[4], const Eigen::Vector3d& v, double w = 0.0) { fill(out, v.cast<float>().eval(), static_cast<float>(w)); }
void fill(float (&out)[16], const Eigen::Matrix4d& m)
{
  const Eigen::Matrix4f f = m.cast<float>();
  std::copy(f.data(), f.data() + 16, out);
}

// Memoria de contacto libre: (0, 0, 0, −1) por hueco
std::vector<GpuVec4> free_memory(size_t points)
{
  return std::vector<GpuVec4>(core::kMaxContactBodies * points, GpuVec4{{0.f, 0.f, 0.f, -1.f}});
}

void clear(const gl::Buffer& buf, GLenum internal, GLenum format, GLenum type, const void* value)
{
  glBindBuffer(GL_COPY_WRITE_BUFFER, buf.id());
  glClearBufferData(GL_COPY_WRITE_BUFFER, internal, format, type, value);
  glBindBuffer(GL_COPY_WRITE_BUFFER, 0);
}

void copy(const gl::Buffer& from, const gl::Buffer& to, GLsizeiptr bytes, GLintptr to_offset = 0)
{
  glBindBuffer(GL_COPY_READ_BUFFER, from.id());
  glBindBuffer(GL_COPY_WRITE_BUFFER, to.id());
  glCopyBufferSubData(GL_COPY_READ_BUFFER, GL_COPY_WRITE_BUFFER, 0, to_offset, bytes);
}
}  // namespace

GpuXpbd::GpuXpbd(gl::GlComputeContext& gl, const core::GpuLayout& L)
: gl_(gl),
  nodes_(static_cast<GLuint>(L.node_count())),
  links_(static_cast<GLuint>(L.link_count())),
  color_begin_(L.color_begin),
  excluded_now_(L.node_count(), 0u)
{
  forces_ = load("forces.comp");
  predict_ = load("predict.comp");
  links_prog_ = load("links.comp");
  anchors_prog_ = load("anchors.comp");
  contact_nodes_ = load("contact_nodes.comp");
  contact_links_ = load("contact_links.comp");
  finish_ = load("finish.comp");
  reduce_ = load("reduce.comp");

  // Las posiciones viven en memoria de la GPU; read_back las copia a x_read_, que está mapeado para la CPU (un
  // buffer mapeado lo pone el driver en memoria del sistema, y los shaders irían por PCIe en cada acceso)
  const std::vector<GpuVec4> zeros(L.node_count(), GpuVec4{});
  x_ = buffer_of(L.x0);
  x_read_ = gl::Buffer(bytes_of(L.x0), nullptr, true);
  v_ = buffer_of(zeros);
  dx_ = buffer_of(zeros);
  vnew_ = buffer_of(zeros);
  node_props_ = buffer_of(L.node_props);
  link_buf_ = buffer_of(L.links);
  link_props_ = buffer_of(L.link_props);
  adj_begin_ = buffer_of(L.adj_begin);
  adj_links_ = buffer_of(L.adj_links);

  excluded_ = buffer_of(excluded_now_);
  node_mem_ = buffer_of(free_memory(L.node_count()));
  link_mem_ = buffer_of(free_memory(L.link_count()));
  node_mem_used_ = buffer_of(std::vector<int32_t>(L.node_count(), 0));
  link_mem_used_ = buffer_of(std::vector<int32_t>(L.link_count(), 0));
  log_ = gl::Buffer(static_cast<GLsizeiptr>(sizeof(GpuMoved)) * kLogCapacity, nullptr);
  log_count_ = buffer_of(std::vector<GLuint>{0});
  params_ = gl::Buffer(sizeof(GpuParams), nullptr);
  jets_ = gl::Buffer(sizeof(GpuJets), nullptr);
  set_jets({}, 0.1f);
  set_anchors({});
  set_bodies({}, 1);
  set_current_grid(Eigen::Vector3f::Zero(), 1.f, 1, 1, 1, {Eigen::Vector3f::Zero()});
}

GpuXpbd::~GpuXpbd()
{
  // Los buffers se borran solos (gl::Buffer); la textura, con la misma regla: solo con su contexto activo
  if (gl_.is_current()) {
    glDeleteTextures(1, &current_);
  }
}

void GpuXpbd::set_current_grid(const Eigen::Vector3f& origin, float cell, int nx, int ny, int nz,
                               const std::vector<Eigen::Vector3f>& v)
{
  if (current_ == 0) {
    glGenTextures(1, &current_);
  }
  glBindTexture(GL_TEXTURE_3D, current_);
  glTexImage3D(GL_TEXTURE_3D, 0, GL_RGB32F, nx, ny, nz, 0, GL_RGB, GL_FLOAT, v.data());
  glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  for (const GLenum wrap : {GL_TEXTURE_WRAP_S, GL_TEXTURE_WRAP_T, GL_TEXTURE_WRAP_R}) {
    glTexParameteri(GL_TEXTURE_3D, wrap, GL_CLAMP_TO_EDGE);
  }
  glBindTexture(GL_TEXTURE_3D, 0);
  current_origin_ = origin;
  current_cell_ = cell;
}

void GpuXpbd::set_jets(const std::vector<core::Jet>& jets, float spread)
{
  GpuJets g{};
  g.count = static_cast<uint32_t>(std::min(jets.size(), core::kMaxJets));
  g.spread = spread;
  for (uint32_t k = 0; k < g.count; ++k) {
    const core::Jet& j = jets[k];
    std::copy(j.origin.data(), j.origin.data() + 3, g.origin_speed[k]);
    g.origin_speed[k][3] = j.speed;
    std::copy(j.dir.data(), j.dir.data() + 3, g.dir_radius[k]);
    g.dir_radius[k][3] = j.radius;
  }
  jets_.write(std::vector<GpuJets>{g});
}

void GpuXpbd::set_anchors(const std::vector<core::Anchor>& anchors)
{
  std::vector<GpuAnchor> data(anchors.size());
  for (size_t j = 0; j < anchors.size(); ++j) {
    const core::Anchor& a = anchors[j];
    fill(data[j].from, a.from);
    fill(data[j].to, a.to);
    data[j].node[0] = a.node;
    for (int c = 0; c < 3; ++c) fill(data[j].inv_mass[c], a.inv_mass.col(c).eval());
  }
  anchor_count_ = static_cast<GLuint>(anchors.size());
  if (grow(anchor_capacity_, anchor_count_)) {
    const GLsizeiptr cap = anchor_capacity_;
    anchors_ = gl::Buffer(static_cast<GLsizeiptr>(sizeof(GpuAnchor)) * cap, nullptr);
    anchor_dx_ = buffer_of(std::vector<GpuVec4>(anchor_capacity_, GpuVec4{}));
    // Para leer: lo acumulado por cada anclaje y, detrás, el estado de su ojal
    anchor_dx_read_ = gl::Buffer(static_cast<GLsizeiptr>(sizeof(GpuVec4) + sizeof(GpuAnchorState)) * cap, nullptr, true);
    // anchors.comp lo pone a cero en el primer subpaso
    anchor_state_ = gl::Buffer(static_cast<GLsizeiptr>(sizeof(GpuAnchorState)) * cap, nullptr);
  }
  anchors_.write(data);
}

void GpuXpbd::set_bodies(const std::vector<core::ContactBody>& bodies, int n)
{
  // BodyData de cada subpaso: BODIES[i * body_count + b]
  std::vector<GpuBody> data;
  data.reserve(static_cast<size_t>(n) * bodies.size());
  for (int i = 0; i < n; ++i) {
    const double s = static_cast<double>(i + 1) / n;
    for (const auto& b : bodies) {
      const Eigen::Isometry3d pose = core::interpolate_pose(b.pose0, b.pose1, s);
      GpuBody d;
      fill(d.to_local, pose.inverse().matrix());
      fill(d.to_world, pose.matrix());
      fill(d.half_rounding, b.box.half_extents, b.box.rounding);
      fill(d.offset_friction, b.box.center_offset, static_cast<double>(b.friction));
      fill(d.cg, ((1.0 - s) * b.cg0 + s * b.cg1).eval());
      fill(d.v_cg, b.v_cg);
      fill(d.w, b.w);
      data.push_back(d);
    }
  }
  body_count_ = static_cast<GLuint>(bodies.size());
  if (grow(body_slots_, body_count_ * static_cast<GLuint>(n))) {
    bodies_ = gl::Buffer(static_cast<GLsizeiptr>(sizeof(GpuBody)) * body_slots_, nullptr);
  }
  bodies_.write(data);
  if (grow(body_capacity_, body_count_)) {
    reactions_ = gl::Buffer(static_cast<GLsizeiptr>(sizeof(GpuMoved)) * body_capacity_, nullptr);
    // detrás de las reacciones, el contador del registro
    reactions_read_ = gl::Buffer(static_cast<GLsizeiptr>(sizeof(GpuMoved)) * body_capacity_ + 16, nullptr, true);
  }

  // Cuerpos que no toca cada nudo (el orinque su boya, lo agarrado los dedos): se suben solo si cambian
  std::vector<uint32_t> excluded = core::exclusion_masks(bodies, nodes_);
  if (excluded != excluded_now_) {
    excluded_now_ = std::move(excluded);
    excluded_.write(excluded_now_);
  }

  // Sin cuerpos no se lanza el contacto: la memoria se vacía aquí, como la vaciaría un subpaso sin contactos
  if (body_count_ == 0 && memory_dirty_) {
    clear_contact_memory();
  }
}

void GpuXpbd::clear_contact_memory()
{
  const float free[4] = {0.f, 0.f, 0.f, -1.f};
  clear(node_mem_, GL_RGBA32F, GL_RGBA, GL_FLOAT, free);
  clear(link_mem_, GL_RGBA32F, GL_RGBA, GL_FLOAT, free);
  const GLint none = 0;
  clear(node_mem_used_, GL_R32I, GL_RED_INTEGER, GL_INT, &none);
  clear(link_mem_used_, GL_R32I, GL_RED_INTEGER, GL_INT, &none);
  memory_dirty_ = false;
}

GpuXpbd::Program GpuXpbd::load(const std::string& file)
{
  static const std::string defines = shader_defines();
  Program p;
  p.id = gl_.program(file, defines);
  auto loc = [&](const char* name) { return glGetUniformLocation(p.id, name); };
  p.n = loc("n");
  p.first = loc("first");
  p.count = loc("count");
  p.body_count = loc("body_count");
  p.substep = loc("substep");
  p.substeps = loc("substeps");
  p.log_capacity = loc("log_capacity");
  // glProgramUniform: sin cambiar el programa activo; con ubicación −1 no hace nada
  glProgramUniform1i(p.id, loc("current"), 0);  // unidad de textura de la corriente
  glProgramUniform1ui(p.id, p.log_capacity, kLogCapacity);
  return p;
}

void GpuXpbd::upload_params(const core::XpbdParams& p)
{
  GpuParams g{};
  g.h = p.h;
  g.rho = p.rho;
  g.g = p.g;
  g.cn = p.cn;
  g.ct = p.ct;
  g.nu = p.nu;
  g.damping = p.damping;
  g.damping_time = p.damping_time;
  g.friction = p.friction;
  g.floor_z = p.floor_z;
  std::copy(current_origin_.data(), current_origin_.data() + 3, g.current_origin);
  g.current_cell = current_cell_;
  if (uploaded_ && std::memcmp(&g, &*uploaded_, sizeof(g)) == 0) {
    return;
  }
  params_.write(std::vector<GpuParams>{g});
  uploaded_ = g;
}

void GpuXpbd::bind_for_tick()
{
  const std::pair<Binding, const gl::Buffer*> buffers[] = {
    {kPositions, &x_},          {kVelocities, &v_},          {kDisplacements, &dx_},      {kNewVelocities, &vnew_},
    {kNodeProps, &node_props_}, {kLinks, &link_buf_},        {kLinkProps, &link_props_},  {kAdjacencyBegin, &adj_begin_},
    {kAdjacencyLinks, &adj_links_}, {kAnchors, &anchors_},   {kAnchorMoved, &anchor_dx_}, {kBodies, &bodies_},
    {kExcluded, &excluded_},    {kNodeMemory, &node_mem_},   {kLinkMemory, &link_mem_},   {kReactionLog, &log_},
    {kReactionCount, &log_count_}, {kReactions, &reactions_}, {kNodeMemoryUsed, &node_mem_used_},
    {kLinkMemoryUsed, &link_mem_used_}, {kAnchorState, &anchor_state_},
  };
  for (const auto& [binding, buf] : buffers) glBindBufferBase(GL_SHADER_STORAGE_BUFFER, binding, buf->id());
  glBindBufferBase(GL_UNIFORM_BUFFER, kParamsBinding, params_.id());
  glBindBufferBase(GL_UNIFORM_BUFFER, kJetsBinding, jets_.id());
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_3D, current_);
  for (const Program* prog : {&forces_, &predict_, &contact_nodes_, &finish_}) glProgramUniform1ui(prog->id, prog->n, nodes_);
  glProgramUniform1ui(anchors_prog_.id, anchors_prog_.n, anchor_count_);
}

void GpuXpbd::dispatch(const Program& prog, GLuint threads)
{
  glUseProgram(prog.id);
  if (threads > 0) {
    glDispatchCompute((threads + kGroup - 1) / kGroup, 1, 1);
  }
  glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
}

void GpuXpbd::dispatch_colors(const Program& prog)
{
  glUseProgram(prog.id);
  for (size_t c = 0; c + 1 < color_begin_.size(); ++c) {
    const auto begin = static_cast<GLuint>(color_begin_[c]), size = static_cast<GLuint>(color_begin_[c + 1]) - begin;
    glUniform1ui(prog.first, begin);
    glUniform1ui(prog.count, size);
    glDispatchCompute((size + kGroup - 1) / kGroup, 1, 1);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
  }
}

void GpuXpbd::substep(const core::XpbdParams& p, int i, int n)
{
  if (!timing_) {
    timer_.begin();
    timing_ = true;
  }
  // Cada llamada a OpenGL cuesta CPU, más en el simulador, donde el hilo de dibujo usa el mismo controlador: lo que
  // no cambia en el tick se pone una vez. Entre ticks, set_anchors y set_bodies pueden rehacer buffers.
  if (i == 0) {
    bind_for_tick();
  }
  upload_params(p);
  dispatch(forces_, nodes_);
  dispatch(predict_, nodes_);
  dispatch_colors(links_prog_);
  glProgramUniform1ui(anchors_prog_.id, anchors_prog_.substep, static_cast<GLuint>(i));
  glProgramUniform1ui(anchors_prog_.id, anchors_prog_.substeps, static_cast<GLuint>(n));
  dispatch(anchors_prog_, anchor_count_);
  if (body_count_ > 0) {
    for (const Program* prog : {&contact_nodes_, &contact_links_}) {
      glProgramUniform1ui(prog->id, prog->body_count, body_count_);
      glProgramUniform1ui(prog->id, prog->substep, static_cast<GLuint>(i));
    }
    dispatch(contact_nodes_, nodes_);
    dispatch_colors(contact_links_);
    memory_dirty_ = true;
  }
  dispatch(finish_, nodes_);
}

void GpuXpbd::read_back(std::vector<Eigen::Vector3f>& x, std::vector<Eigen::Vector3f>& anchor_dx,
                        std::vector<Eigen::Vector3f>& anchor_offset, std::vector<core::BodyReaction>& reactions)
{
  if (timing_) {
    timer_.end();
    timing_ = false;
  }
  glMemoryBarrier(GL_BUFFER_UPDATE_BARRIER_BIT);  // lo escrito por los shaders, antes de copiarlo
  copy(x_, x_read_, static_cast<GLsizeiptr>(sizeof(GpuVec4)) * nodes_);
  const auto moved_bytes = static_cast<GLsizeiptr>(sizeof(GpuVec4)) * anchor_count_;
  if (anchor_count_ > 0) {
    copy(anchor_dx_, anchor_dx_read_, moved_bytes);
    copy(anchor_state_, anchor_dx_read_, static_cast<GLsizeiptr>(sizeof(GpuAnchorState)) * anchor_count_, moved_bytes);
  }
  const auto reaction_bytes = static_cast<GLsizeiptr>(sizeof(GpuMoved)) * body_capacity_;
  if (body_count_ > 0) {
    glUseProgram(reduce_.id);
    glDispatchCompute(body_count_, 1, 1);
    glMemoryBarrier(GL_BUFFER_UPDATE_BARRIER_BIT);
    copy(reactions_, reactions_read_, static_cast<GLsizeiptr>(sizeof(GpuMoved)) * body_count_);
    copy(log_count_, reactions_read_, sizeof(GLuint), reaction_bytes);
  }
  gl::wait_gpu();

  const auto* xs = reinterpret_cast<const GpuVec4*>(x_read_.mapped());
  x.resize(nodes_);
  for (GLuint k = 0; k < nodes_; ++k) x[k] = Eigen::Vector3f(xs[k].v[0], xs[k].v[1], xs[k].v[2]);
  const auto* moved = reinterpret_cast<const GpuVec4*>(anchor_dx_read_.mapped());
  const auto* state = reinterpret_cast<const GpuAnchorState*>(moved + anchor_count_);  // detrás de lo acumulado
  anchor_dx.resize(anchor_count_);
  anchor_offset.resize(anchor_count_);
  for (GLuint j = 0; j < anchor_count_; ++j) {
    anchor_dx[j] = Eigen::Vector3f(moved[j].v[0], moved[j].v[1], moved[j].v[2]);
    anchor_offset[j] = Eigen::Vector3f(state[j].offset[0], state[j].offset[1], state[j].offset[2]);
  }
  reactions.assign(body_count_, core::BodyReaction{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()});
  lost_contacts_ = 0;
  if (body_count_ > 0) {
    const auto* r = reinterpret_cast<const GpuMoved*>(reactions_read_.mapped());
    for (GLuint b = 0; b < body_count_; ++b) {
      reactions[b].momentum = Eigen::Vector3d(r[b].momentum[0], r[b].momentum[1], r[b].momentum[2]);
      reactions[b].angular = Eigen::Vector3d(r[b].angular[0], r[b].angular[1], r[b].angular[2]);
    }
    GLuint logged = 0;
    std::memcpy(&logged, reinterpret_cast<const char*>(reactions_read_.mapped()) + reaction_bytes, sizeof(logged));
    lost_contacts_ = logged > kLogCapacity ? logged - kLogCapacity : 0;
  }

  const float zero = 0.f;
  if (anchor_count_ > 0) {
    clear(anchor_dx_, GL_R32F, GL_RED, GL_FLOAT, &zero);
  }
  const GLuint none = 0;
  clear(log_count_, GL_R32UI, GL_RED_INTEGER, GL_UNSIGNED_INT, &none);
}

}  // namespace stonefish_net::gpu
