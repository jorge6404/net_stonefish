// Contexto de OpenGL propio para calcular la red con compute shaders (GLSL 430) en el hilo de la física.
//
// Stonefish dibuja en su hilo con su contexto, y la física corre en otro hilo sin ninguno. Este contexto se crea
// sobre la ventana de SDL de Stonefish, ya en el hilo de la física (ensure_current), para que caiga en la misma GPU
// que el dibujo; no comparte objetos con el de Stonefish. Solo usa SDL2 y el glad de Stonefish, no el resto de
// Stonefish: también lo usa tools/gpu_check, con su propia ventana oculta.
#pragma once

#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <SDL2/SDL.h>
#include <glad.h>

namespace stonefish_net::gl
{

class GlComputeContext
{
public:
  // window: la ventana de SDL donde crear el contexto. No toca GL aquí.
  // shader_dir: carpeta de los .comp (share/stonefish_net/shaders).
  GlComputeContext(SDL_Window* window, std::string shader_dir);
  ~GlComputeContext();
  GlComputeContext(const GlComputeContext&) = delete;
  GlComputeContext& operator=(const GlComputeContext&) = delete;

  // La primera vez, en el hilo que lo llama: crea el contexto (4.3 core), lo activa, carga glad si nadie lo ha
  // cargado y comprueba que hay OpenGL >= 4.3. Las demás, nada. Lanza std::runtime_error con el motivo.
  void ensure_current();

  // Programa de cálculo de shader_dir/<file>, compilado y enlazado una vez por cada `defines`. Las líneas
  // `#include "otro.glsl"` se sustituyen por ese fichero y `defines` (líneas `#define NOMBRE valor`) va detrás de
  // `#version`: así las constantes que comparten C++ y GLSL tienen un solo origen. Lanza std::runtime_error con el
  // fichero y el log del compilador.
  GLuint program(const std::string& file, const std::string& defines = "");

  std::string vendor() const;  // "GL_VENDOR / GL_RENDERER / GL_VERSION"

  // Si este contexto es el activo en el hilo que lo pregunta
  bool is_current() const { return context_ != nullptr && SDL_GL_GetCurrentContext() == context_; }

private:
  SDL_Window* window_;
  std::string shader_dir_;
  SDL_GLContext context_ = nullptr;
  std::thread::id owner_;  // el hilo donde se creó y está activo
  std::unordered_map<std::string, GLuint> programs_;
};

// Buffer de OpenGL de `bytes` bytes y, si data no es nulo, ese contenido; se puede reescribir con glBufferSubData. Con
// readback queda mapeado de forma persistente y coherente para leerlo desde la CPU (mapped). Nunca de tamaño 0:
// reserva al menos 16 bytes.
//
// Se borra al destruirse solo si el contexto en que se creó es el activo en ese hilo. Si no (fin del escenario en
// otro hilo), lo libera ese contexto al borrarse; y otro contexto activo, el de dibujo de Stonefish, tiene nombres de
// buffer que coinciden con los nuestros: borrarlos ahí rompería su dibujo.
class Buffer
{
public:
  Buffer() = default;
  Buffer(GLsizeiptr bytes, const void* data, bool readback = false);
  ~Buffer() { release(); }
  Buffer(Buffer&& other) noexcept { *this = std::move(other); }
  Buffer& operator=(Buffer&& other) noexcept;
  Buffer(const Buffer&) = delete;
  Buffer& operator=(const Buffer&) = delete;

  GLuint id() const { return id_; }
  const float* mapped() const { return static_cast<const float*>(mapped_); }

  // Reescribe los primeros bytes de data.size() elementos
  template <typename T>
  void write(const std::vector<T>& data)
  {
    if (data.empty()) return;
    glBindBuffer(GL_COPY_WRITE_BUFFER, id_);
    glBufferSubData(GL_COPY_WRITE_BUFFER, 0, static_cast<GLsizeiptr>(data.size() * sizeof(T)), data.data());
    glBindBuffer(GL_COPY_WRITE_BUFFER, 0);
  }

private:
  void release();

  GLuint id_ = 0;
  SDL_GLContext context_ = nullptr;
  void* mapped_ = nullptr;
};

// Espera a que la GPU acabe lo lanzado hasta ahora y deja visibles a la CPU las escrituras en buffers mapeados.
void wait_gpu();

// Tiempo de GPU de un tramo de trabajo (GL_TIME_ELAPSED). Dos consultas alternas: last_ms() da la del tick
// anterior sin esperar a la GPU.
class GpuTimer
{
public:
  void begin();
  void end();
  double last_ms();

private:
  GLuint queries_[2] = {0, 0};
  int current_ = 0;
  bool pending_[2] = {false, false};
  double last_ms_ = 0.0;
};

}  // namespace stonefish_net::gl
