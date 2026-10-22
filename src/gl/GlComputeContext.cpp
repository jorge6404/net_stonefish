#include "stonefish_net/gl/GlComputeContext.h"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace stonefish_net::gl
{
namespace
{
std::string read_file(const std::string& path)
{
  std::ifstream f(path);
  if (!f) {
    throw std::runtime_error("no se encuentra el shader '" + path + "'");
  }
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

// GLSL no tiene includes: cada línea `#include "fichero"` se sustituye por ese fichero de la misma carpeta
// (un nivel, sin anidar)
std::string resolve_includes(const std::string& source, const std::string& dir)
{
  std::istringstream in(source);
  std::string out, line;
  while (std::getline(in, line)) {
    const auto start = line.find("#include \"");
    if (start != std::string::npos && line.find_first_not_of(" \t") == start) {
      const auto open = line.find('"'), close = line.rfind('"');
      out += read_file(dir + "/" + line.substr(open + 1, close - open - 1));
    } else {
      out += line;
    }
    out += '\n';
  }
  return out;
}

const char* gl_string(GLenum name)
{
  const auto* s = reinterpret_cast<const char*>(glGetString(name));
  return s ? s : "?";
}
}  // namespace

GlComputeContext::GlComputeContext(SDL_Window* window, std::string shader_dir)
: window_(window), shader_dir_(std::move(shader_dir))
{
}

GlComputeContext::~GlComputeContext()
{
  if (context_ == nullptr) {
    return;
  }
  // Desde otro hilo no se puede llamar a GL con este contexto: borrarlo libera todos sus objetos.
  if (std::this_thread::get_id() == owner_) {
    for (const auto& [name, prog] : programs_) glDeleteProgram(prog);
    SDL_GL_MakeCurrent(window_, nullptr);
  }
  SDL_GL_DeleteContext(context_);
}

void GlComputeContext::ensure_current()
{
  if (context_ != nullptr) {
    return;
  }
  if (window_ == nullptr) {
    throw std::runtime_error("no hay ventana de SDL donde crear el contexto de OpenGL");
  }
  // Stonefish deja activo compartir con el contexto actual; este no comparte nada (en este hilo no hay ninguno)
  int share = 0;
  SDL_GL_GetAttribute(SDL_GL_SHARE_WITH_CURRENT_CONTEXT, &share);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_SHARE_WITH_CURRENT_CONTEXT, 0);
  context_ = SDL_GL_CreateContext(window_);
  SDL_GL_SetAttribute(SDL_GL_SHARE_WITH_CURRENT_CONTEXT, share);
  if (context_ == nullptr) {
    throw std::runtime_error(std::string("no se puede crear el contexto de OpenGL 4.3: ") + SDL_GetError());
  }
  if (SDL_GL_MakeCurrent(window_, context_) != 0) {
    throw std::runtime_error(std::string("no se puede activar el contexto de OpenGL: ") + SDL_GetError());
  }
  owner_ = std::this_thread::get_id();
  // Los punteros de glad son globales y valen para cualquier contexto del mismo controlador; Stonefish ya los
  // ha cargado, gpu_check no.
  if (glad_glDispatchCompute == nullptr &&
      gladLoadGL(reinterpret_cast<GLADloadfunc>(SDL_GL_GetProcAddress)) == 0) {
    throw std::runtime_error("no se pueden cargar las funciones de OpenGL");
  }
  GLint major = 0, minor = 0;
  glGetIntegerv(GL_MAJOR_VERSION, &major);
  glGetIntegerv(GL_MINOR_VERSION, &minor);
  if (major < 4 || (major == 4 && minor < 3)) {
    throw std::runtime_error("OpenGL " + std::to_string(major) + "." + std::to_string(minor) +
                             " < 4.3 (sin compute shaders) en " + vendor());
  }
}

GLuint GlComputeContext::program(const std::string& file, const std::string& defines)
{
  const std::string key = file + "\n" + defines;
  if (const auto it = programs_.find(key); it != programs_.end()) {
    return it->second;
  }
  const std::string path = shader_dir_ + "/" + file;
  std::string source = resolve_includes(read_file(path), shader_dir_);
  const size_t version_end = source.find('\n', source.find("#version"));
  if (version_end == std::string::npos) {
    throw std::runtime_error("el shader '" + path + "' no empieza por #version");
  }
  source.insert(version_end + 1, defines);
  const char* src = source.c_str();
  const GLuint shader = glCreateShader(GL_COMPUTE_SHADER);
  glShaderSource(shader, 1, &src, nullptr);
  glCompileShader(shader);
  auto log_of = [](GLuint obj, bool is_program) {
    GLint len = 0;
    is_program ? glGetProgramiv(obj, GL_INFO_LOG_LENGTH, &len) : glGetShaderiv(obj, GL_INFO_LOG_LENGTH, &len);
    std::vector<char> log(static_cast<size_t>(std::max(len, 1)), '\0');
    is_program ? glGetProgramInfoLog(obj, len, nullptr, log.data()) : glGetShaderInfoLog(obj, len, nullptr, log.data());
    return std::string(log.data());
  };
  GLint ok = 0;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
  if (!ok) {
    const std::string log = log_of(shader, false);
    glDeleteShader(shader);
    throw std::runtime_error("el shader '" + path + "' no compila:\n" + log);
  }
  const GLuint prog = glCreateProgram();
  glAttachShader(prog, shader);
  glLinkProgram(prog);
  glDeleteShader(shader);
  glGetProgramiv(prog, GL_LINK_STATUS, &ok);
  if (!ok) {
    const std::string log = log_of(prog, true);
    glDeleteProgram(prog);
    throw std::runtime_error("el shader '" + path + "' no enlaza:\n" + log);
  }
  programs_[key] = prog;
  return prog;
}

std::string GlComputeContext::vendor() const
{
  return std::string(gl_string(GL_VENDOR)) + " / " + gl_string(GL_RENDERER) + " / " + gl_string(GL_VERSION);
}

Buffer::Buffer(GLsizeiptr bytes, const void* data, bool readback) : context_(SDL_GL_GetCurrentContext())
{
  bytes = std::max<GLsizeiptr>(bytes, 16);
  glGenBuffers(1, &id_);
  glBindBuffer(GL_COPY_WRITE_BUFFER, id_);
  const GLbitfield map = readback ? GL_MAP_READ_BIT | GL_MAP_PERSISTENT_BIT | GL_MAP_COHERENT_BIT : 0;
  glBufferStorage(GL_COPY_WRITE_BUFFER, bytes, data, map | GL_DYNAMIC_STORAGE_BIT);
  if (readback) {
    mapped_ = glMapBufferRange(GL_COPY_WRITE_BUFFER, 0, bytes, map);
  }
  glBindBuffer(GL_COPY_WRITE_BUFFER, 0);
}

Buffer& Buffer::operator=(Buffer&& other) noexcept
{
  if (this != &other) {
    release();
    id_ = std::exchange(other.id_, 0);
    context_ = std::exchange(other.context_, nullptr);
    mapped_ = std::exchange(other.mapped_, nullptr);
  }
  return *this;
}

void Buffer::release()
{
  if (id_ != 0 && SDL_GL_GetCurrentContext() == context_) {
    glDeleteBuffers(1, &id_);
  }
  id_ = 0;
  mapped_ = nullptr;
}

void wait_gpu()
{
  glMemoryBarrier(GL_CLIENT_MAPPED_BUFFER_BARRIER_BIT);
  GLsync fence = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
  while (glClientWaitSync(fence, GL_SYNC_FLUSH_COMMANDS_BIT, 1'000'000'000) == GL_TIMEOUT_EXPIRED) {
  }
  glDeleteSync(fence);
}

void GpuTimer::begin()
{
  if (queries_[0] == 0) {
    glGenQueries(2, queries_);
  }
  glBeginQuery(GL_TIME_ELAPSED, queries_[current_]);
}

void GpuTimer::end()
{
  glEndQuery(GL_TIME_ELAPSED);
  pending_[current_] = true;
  current_ ^= 1;
}

double GpuTimer::last_ms()
{
  // La consulta que no está en curso es la del tick anterior
  const int other = current_;
  if (pending_[other]) {
    GLint available = 0;
    glGetQueryObjectiv(queries_[other], GL_QUERY_RESULT_AVAILABLE, &available);
    if (available) {
      GLuint64 ns = 0;
      glGetQueryObjectui64v(queries_[other], GL_QUERY_RESULT, &ns);
      last_ms_ = static_cast<double>(ns) * 1e-6;
      pending_[other] = false;
    }
  }
  return last_ms_;
}

}  // namespace stonefish_net::gl
