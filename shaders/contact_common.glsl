// Contacto de la red con cajas redondeadas (sólidos): la geometría de core/contact y la corrección de
// core/xpbd_ref (contact_point), en float. Se incluye después de xpbd_common.glsl.

// Un cuerpo en un subpaso: BODIES[substep * body_count + b]
struct BodyData
{
  mat4 to_local;        // mundo → marco del cuerpo
  mat4 to_world;        // marco del cuerpo → mundo
  vec4 half_rounding;   // semiejes exteriores, radio de redondeo
  vec4 offset_friction; // centro de la caja en el marco del cuerpo, μ
  vec4 cg;              // centro de gravedad en este subpaso
  vec4 v_cg;            // velocidad del centro de gravedad
  vec4 w;               // velocidad angular
};
layout(std430, binding = B_BODIES) readonly buffer Bodies { BodyData BODIES[]; };
layout(std430, binding = B_EXCLUDED) readonly buffer Excluded { uint EXCLUDED[]; };  // por nudo: bit b = no toca el cuerpo b
layout(std430, binding = B_NODE_MEMORY) buffer NodeMemory { vec4 NODE_MEM[]; };       // MAX_BODIES por nudo
layout(std430, binding = B_LINK_MEMORY) buffer LinkMemory { vec4 LINK_MEM[]; };       // MAX_BODIES por tramo
// Huecos ocupados de la memoria de cada punto: casi ningún punto toca nada, y así no se leen ni escriben sus
// 4 huecos en cada subpaso (sería el grueso del tráfico de memoria del contacto)
layout(std430, binding = B_NODE_MEMORY_USED) buffer NodeMemoryUsed { int NODE_MEM_USED[]; };
layout(std430, binding = B_LINK_MEMORY_USED) buffer LinkMemoryUsed { int LINK_MEM_USED[]; };
// Registro de lo que la red movió por cada cuerpo (momentum.w = cuerpo); reduce.comp lo suma al leer
struct LogEntry { vec4 momentum; vec4 angular; };
layout(std430, binding = B_REACTION_LOG) buffer ReactionLog { LogEntry LOG[]; };
layout(std430, binding = B_REACTION_COUNT) buffer ReactionCount { uint LOG_COUNT; };

uniform uint body_count;
uniform uint log_capacity;

// Si el nudo k no toca el cuerpo b (core::excludes: solo los MAX_EXCLUDING_BODIES primeros cuerpos pueden excluir)
bool excluded(uint k, uint b)
{
  return b < uint(MAX_EXCLUDING_BODIES) && (EXCLUDED[k] & (1u << b)) != 0u;
}

const float END = 1e-3;     // punto más hundido en un extremo del tramo: ya lo trata el nudo

// Distancia con signo a la caja redondeada de un punto en el marco de la caja (centrado): q = |local| − interior
float local_signed_distance(vec4 hr, vec3 local, out vec3 q)
{
  const vec3 inner = max(hr.xyz - vec3(hr.w), vec3(1e-4));
  q = abs(local) - inner;
  return length(max(q, vec3(0.0))) + min(max(q.x, max(q.y, q.z)), 0.0) - hr.w;
}

vec3 to_box(BodyData B, vec3 p) { return (B.to_local * vec4(p, 1.0)).xyz - B.offset_friction.xyz; }

bool point_in_box_bounds(BodyData B, vec3 p)
{
  const vec3 d = abs(to_box(B, p)) - B.half_rounding.xyz;
  return max(d.x, max(d.y, d.z)) <= 0.0;
}

bool segment_in_box_bounds(BodyData B, vec3 p0, vec3 p1)
{
  const vec3 a = to_box(B, p0);
  const vec3 d = to_box(B, p1) - a;
  float lo = 0.0, hi = 1.0;
  for (int axis = 0; axis < 3; ++axis) {
    const float h = B.half_rounding[axis];
    if (abs(d[axis]) < 1e-12) {
      if (abs(a[axis]) > h) return false;
      continue;
    }
    float t0 = (-h - a[axis]) / d[axis], t1 = (h - a[axis]) / d[axis];
    if (t0 > t1) { const float t = t0; t0 = t1; t1 = t; }
    lo = max(lo, t0);
    hi = min(hi, t1);
    if (lo > hi) return false;
  }
  return true;
}

// Parámetro del punto más hundido del segmento (sección áurea, 20 pasos)
float deepest_on_segment(BodyData B, vec3 p0, vec3 p1)
{
  const vec3 a = to_box(B, p0);
  const vec3 d = to_box(B, p1) - a;
  vec3 q;
  const float inv_phi = 0.6180339887498949;
  float lo = 0.0, hi = 1.0;
  float x1 = hi - inv_phi * (hi - lo), x2 = lo + inv_phi * (hi - lo);
  float f1 = local_signed_distance(B.half_rounding, a + x1 * d, q);
  float f2 = local_signed_distance(B.half_rounding, a + x2 * d, q);
  for (int it = 0; it < 20; ++it) {
    if (f1 <= f2) {
      hi = x2; x2 = x1; f2 = f1;
      x1 = hi - inv_phi * (hi - lo);
      f1 = local_signed_distance(B.half_rounding, a + x1 * d, q);
    } else {
      lo = x1; x1 = x2; f1 = f2;
      x2 = lo + inv_phi * (hi - lo);
      f2 = local_signed_distance(B.half_rounding, a + x2 * d, q);
    }
  }
  return 0.5 * (lo + hi);
}

// Contacto de un punto: por la superficie más cercana (along = false) o saliendo en la dirección exit_local.
// Devuelve false si está fuera.
bool box_contact(BodyData B, vec3 p, bool along, vec3 exit_local, out float depth, out vec3 normal, out vec3 surface)
{
  const vec3 local = to_box(B, p);
  vec3 q;
  const float sd = local_signed_distance(B.half_rounding, local, q);
  if (sd >= 0.0) return false;
  vec3 n_local;
  if (along) {
    float lo = 0.0, hi = 4.0 * length(B.half_rounding.xyz);
    for (int it = 0; it < 24; ++it) {
      const float mid = 0.5 * (lo + hi);
      vec3 qq;
      if (local_signed_distance(B.half_rounding, local + mid * exit_local, qq) < 0.0) lo = mid; else hi = mid;
    }
    depth = hi;
    n_local = exit_local;
  } else {
    const vec3 outside = max(q, vec3(0.0));
    const float outside_dist = length(outside);
    const vec3 s = vec3(local.x >= 0.0 ? 1.0 : -1.0, local.y >= 0.0 ? 1.0 : -1.0, local.z >= 0.0 ? 1.0 : -1.0);
    if (outside_dist > 1e-6) {
      n_local = s * (outside / outside_dist);
    } else {
      // por la cara más cercana; en un empate, el primer eje (como Eigen::maxCoeff)
      int axis = 0;
      if (q.y > q[axis]) axis = 1;
      if (q.z > q[axis]) axis = 2;
      n_local = vec3(0.0);
      n_local[axis] = s[axis];
    }
    depth = -sd;
  }
  normal = mat3(B.to_world) * n_local;
  surface = p + depth * normal;
  return true;
}

// Saca de la caja del cuerpo b el punto (1 − u)·x_a + u·x_c (un nudo: a == c), como core::xpbd_ref: por la
// dirección recordada si ya estaba en contacto, si no por la superficie más cercana; Coulomb sobre su
// desplazamiento en el subpaso relativo a la superficie; la corrección se reparte según peso y masa inversa.
// Apunta en el registro lo movido y devuelve en exit_out la dirección de salida (marco de la caja).
bool contact_point(uint b, BodyData B, vec4 old_slots[MAX_BODIES], int a, int c, float u, out vec3 exit_out)
{
  const float wa = 1.0 - u, wb = u;
  const float ima = X[a].w, imb = X[c].w;
  const float share = a == c ? ima : wa * wa * ima + wb * wb * imb;
  if (share <= 0.0) return false;
  const vec3 pt = wa * X[a].xyz + wb * X[c].xyz;
  bool along = false;
  vec3 exit_local = vec3(0.0);
  for (int j = 0; j < MAX_BODIES; ++j) {
    if (int(old_slots[j].w) == int(b)) {
      along = true;
      exit_local = normalize(old_slots[j].xyz);
      break;
    }
  }
  float depth;
  vec3 normal, surface;
  if (!box_contact(B, pt, along, exit_local, depth, normal, surface)) return false;
  const vec3 normal_fix = depth * normal;
  const vec3 disp = wa * DX[a].xyz + wb * DX[c].xyz;
  const vec3 v_surface = B.v_cg.xyz + cross(B.w.xyz, surface - B.cg.xyz);
  const vec3 slip = disp + normal_fix - v_surface * h;
  const vec3 slip_t = slip - dot(slip, normal) * normal;
  const float slip_norm = length(slip_t);
  vec3 fix = normal_fix;
  if (slip_norm > 0.0) fix -= slip_t * min(1.0, B.offset_friction.w * depth / slip_norm);

  vec3 moved = vec3(0.0);
  if (a == c) {
    const vec3 d = fix;
    X[a].xyz += d;
    DX[a].xyz += d;
    moved += d / ima;
  } else {
    if (ima > 0.0) {
      const vec3 d = (wa * ima / share) * fix;
      X[a].xyz += d;
      DX[a].xyz += d;
      moved += d / ima;
    }
    if (imb > 0.0) {
      const vec3 d = (wb * imb / share) * fix;
      X[c].xyz += d;
      DX[c].xyz += d;
      moved += d / imb;
    }
  }
  const uint slot = atomicAdd(LOG_COUNT, 1u);
  if (slot < log_capacity) {
    LOG[slot].momentum = vec4(moved, float(b));
    LOG[slot].angular = vec4(cross(surface - B.cg.xyz, moved), 0.0);
  }
  exit_out = transpose(mat3(B.to_world)) * normal;
  return true;
}

// Guarda en slots la salida de un contacto con el cuerpo b (si queda hueco)
void remember(inout vec4 slots[MAX_BODIES], uint b, vec3 exit_dir)
{
  for (int j = 0; j < MAX_BODIES; ++j) {
    if (slots[j].w < 0.0) {
      slots[j] = vec4(exit_dir, float(b));
      return;
    }
  }
}
