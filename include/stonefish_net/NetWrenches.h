// Fuerza que la red hace sobre cada robot y cada pinza, publicada en cada tick como geometry_msgs/WrenchStamped:
// la reacción que la red acaba de calcular (SolidContacts::pending, con los dos backends) y que el sólido recibe
// durante el tick siguiente, sumada sobre los cuerpos de contacto del grupo y expresada como la mediría un sensor de
// fuerza montado en su marco: la fuerza en sus ejes y el par respecto a su origen.
//
//   /stonefish_net/wrench/<robot>              todos los enlaces del robot con contacto propio (pinza incluida),
//                                              en el marco del enlace base; frame_id <robot>/base_link
//   /stonefish_net/wrench/<robot>/gripper_<k>  los dedos de la pinza k de grippers (en el orden del YAML), en el
//                                              marco de su primer dedo, al que se atan los nudos agarrados;
//                                              frame_id <robot>/<dedo>
//
// El sello es el reloj del nodo, como el de los sensores de stonefish_ros2: cuadra con las imágenes de las cámaras.
#pragma once

#include <string>
#include <vector>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "stonefish_net/SolidContacts.h"
#include "stonefish_net/core/wrench.hpp"

namespace sf
{
class SolidEntity;
}

namespace stonefish_net
{

class NetWrenches
{
public:
  struct Group
  {
    std::string topic, frame_id;
    sf::SolidEntity* frame;       // el marco O de este sólido es el del sensor
    std::vector<size_t> bodies;   // índices en SolidContacts::bodies()
  };

  NetWrenches(rclcpp::Node& node, std::vector<Group> groups);

  // Post-tick, después de NetSolver::end_step
  void publish(const SolidContacts& solids, const rclcpp::Time& stamp);

private:
  std::vector<Group> groups_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr> publishers_;
};

}  // namespace stonefish_net
