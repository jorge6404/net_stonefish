// Común a los kernels del subpaso XPBD de la red (src/gpu/GpuXpbd.cpp). Las mismas cuentas, en el mismo orden,
// que la referencia en C++ (core/xpbd_ref.cpp), con la que tools/gpu_check los compara.
// NED: z hacia abajo. Un hilo por nudo, tramo o anclaje; GROUP_SIZE por grupo.
// Los B_* (bindings), GROUP_SIZE, MAX_BODIES y MAX_EXCLUDING_BODIES los define GpuXpbd.cpp al compilar.
layout(local_size_x = GROUP_SIZE) in;

layout(std430, binding = B_POSITIONS) buffer Positions { vec4 X[]; };        // xyz; w = 1/masa inercial (0: fijo)
layout(std430, binding = B_DISPLACEMENTS) buffer Displacements { vec4 DX[]; };   // desplazamiento del subpaso: v = dx/h
layout(std430, binding = B_LINKS) readonly buffer Links { ivec2 LINKS[]; };
// Los shaders de contacto (#define CONTACT_SHADER) no los declaran: la GPU admite pocos bloques por shader (16
// en la NVIDIA) y contact_common.glsl declara los suyos
#ifndef CONTACT_SHADER
layout(std430, binding = B_VELOCITIES) buffer Velocities { vec4 V[]; };
layout(std430, binding = B_NEW_VELOCITIES) buffer NewVelocities { vec4 VNEW[]; };
layout(std430, binding = B_NODE_PROPS) readonly buffer NodeProps { vec4 NPROPS[]; };  // masa, volumen, masa inercial, Ø nudo
layout(std430, binding = B_LINK_PROPS) readonly buffer LinkProps { vec4 LPROPS[]; };  // longitud natural, diámetro, compliance
layout(std430, binding = B_ADJACENCY_BEGIN) readonly buffer AdjacencyBegin { int ADJ_BEGIN[]; };
layout(std430, binding = B_ADJACENCY_LINKS) readonly buffer AdjacencyLinks { int ADJ_LINKS[]; };
// El ojal al inicio y al final del tick sin la red, y la inversa de la masa de la boya en el ojal (columnas)
struct AnchorData { vec4 from; vec4 to; ivec4 node; vec4 inv_mass[3]; };
layout(std430, binding = B_ANCHORS) readonly buffer Anchors { AnchorData ANCHORS[]; };
layout(std430, binding = B_ANCHOR_MOVED) buffer AnchorMoved { vec4 ANCHOR_DX[]; };  // Σ m·Δx del tick, por anclaje
#endif

uniform uint n;           // nudos (o anclajes, en anchors.comp)
uniform uint first;       // primer tramo del color (links.comp)
uniform uint count;       // tramos del color (links.comp)
uniform uint substep;     // subpaso en curso, y subpasos del tick (anchors.comp)
uniform uint substeps;

// Los parámetros del subpaso (core::XpbdParams) y la rejilla de la corriente: gpu::GpuParams
layout(std140, binding = B_PARAMS) uniform Params
{
  float h;  // s
  float rho, g, cn, ct, nu, damping, damping_time, friction, floor_z;
  vec3 current_origin;
  float current_cell;
};

// Corriente: textura 3D de velocidades del agua sobre una rejilla de lado current_cell que empieza en current_origin
uniform sampler3D current;

// Los chorros de los propulsores (core::Jet): gpu::GpuJets
layout(std140, binding = B_JETS) uniform Jets
{
  vec4 jet_origin_speed[MAX_JETS];  // centro de la hélice, u₀
  vec4 jet_dir_radius[MAX_JETS];    // hacia donde va el agua, r₀
  uint jet_count;
  float jet_spread;
};

const float PI = 3.14159265358979;

// core::jet_velocity
vec3 jet_velocity(vec3 p)
{
  vec3 sum = vec3(0.0);
  for (uint j = 0u; j < jet_count; ++j) {
    const vec3 origin = jet_origin_speed[j].xyz, dir = jet_dir_radius[j].xyz;
    const float speed = jet_origin_speed[j].w, radius = jet_dir_radius[j].w;
    const vec3 r = p - origin;
    const float s = dot(r, dir);
    const float d0 = 2.0 * radius;
    if (s <= 0.0 || s > JET_LENGTH * d0) {
      continue;
    }
    const float radial = length(r - s * dir);
    const float core_len = JET_CORE * d0;
    const float axis = s <= core_len ? speed : speed * core_len / s;
    const float b = max(radius, jet_spread * s);
    sum += axis * exp(-(radial / b) * (radial / b)) * dir;
  }
  return sum;
}

// El agua en p: la corriente del océano más los chorros de los propulsores
vec3 water_at(vec3 p)
{
  const vec3 cells = vec3(textureSize(current, 0));
  return texture(current, ((p - current_origin) / current_cell + 0.5) / cells).xyz + jet_velocity(p);
}
