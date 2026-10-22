// Contacto cinemático y de penalización para cuerpos de la red (NED: z hacia abajo).
#pragma once

#include <optional>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace stonefish_net::core
{

// Contacto de un nudo con el suelo plano z = floor_z (NED: más hondo = z mayor). Si está por debajo,
// aplica una fuerza que en un paso deja su velocidad vertical en −penetración/dt (lo devuelve al suelo;
// nunca tira hacia abajo) y fricción de Coulomb |F_t| <= μ|F_n| que no invierte el deslizamiento.
Eigen::Vector3d floor_force(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, double mass, double floor_z,
                            double dt, double mu);

// Caja con aristas y esquinas redondeadas, en el marco de un cuerpo rígido.
struct RoundedBox
{
  Eigen::Vector3d half_extents;   // semiejes exteriores (incluido el redondeo), m
  double rounding;                // radio de aristas y esquinas, m
  Eigen::Vector3d center_offset;  // centro de la caja en el marco del cuerpo, m
};

struct Contact
{
  double depth;                   // penetración, m (> 0)
  Eigen::Vector3d normal;         // normal saliente, marco mundo
  Eigen::Vector3d surface_point;  // punto de la superficie más cercano, marco mundo
};

// Distancia con signo de un punto a la caja `box` del cuerpo en `pose` (marco del cuerpo → mundo): negativa dentro.
double box_signed_distance(const RoundedBox& box, const Eigen::Isometry3d& pose, const Eigen::Vector3d& p);

// Contacto de un punto con la caja `box` del cuerpo en `pose` (marco del cuerpo → mundo); nada si está fuera.
// Sale por la superficie más cercana.
std::optional<Contact> box_contact(const RoundedBox& box, const Eigen::Isometry3d& pose, const Eigen::Vector3d& p);

// Igual, pero sale en la dirección `exit_local` (unitaria, en el marco de la caja) en vez de por la superficie
// más cercana: así un punto que sigue dentro de una pieza fina sale por la cara por la que entró aunque haya
// pasado del centro. La normal es esa dirección y la profundidad, lo que hay que recorrer en ella hasta salir.
std::optional<Contact> box_contact_along(const RoundedBox& box, const Eigen::Isometry3d& pose, const Eigen::Vector3d& p,
                                         const Eigen::Vector3d& exit_local);

// Descartes rápidos antes de lo anterior: si el punto, o algún punto del segmento, cae dentro de la caja sin
// redondear (la que tiene los semiejes exteriores, que contiene a la redondeada). Falso = seguro que no toca.
bool point_in_box_bounds(const RoundedBox& box, const Eigen::Isometry3d& pose, const Eigen::Vector3d& p);
bool segment_in_box_bounds(const RoundedBox& box, const Eigen::Isometry3d& pose, const Eigen::Vector3d& p0,
                           const Eigen::Vector3d& p1);

// Parámetro t ∈ [0, 1] del punto p0 + t (p1 − p0) más hundido en la caja (el de menor distancia con signo).
// La caja es convexa, así que su distancia con signo es convexa a lo largo del segmento: un único mínimo,
// que se busca por sección áurea. Encuentra el cruce aunque el segmento atraviese la caja de lado a lado sin
// tener ningún extremo dentro.
double deepest_on_segment(const RoundedBox& box, const Eigen::Isometry3d& pose, const Eigen::Vector3d& p0,
                          const Eigen::Vector3d& p1);

}  // namespace stonefish_net::core
