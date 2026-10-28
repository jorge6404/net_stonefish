// Los chorros de los propulsores de los robots sobre la red (core::jet_from_thrust), con su empuje y su marco en el
// momento de llamarlo. Stonefish empuja a lo largo del eje x del marco del actuador.
#pragma once

#include <vector>

#include <Stonefish/actuators/Thruster.h>

#include "convert.h"
#include "stonefish_net/core/jet.hpp"

namespace stonefish_net
{

inline std::vector<core::Jet> thruster_jets(const std::vector<sf::Thruster*>& thrusters, double rho)
{
  std::vector<core::Jet> jets;
  for (sf::Thruster* t : thrusters) {
    if (jets.size() == core::kMaxJets) break;
    const sf::Transform frame = t->getActuatorFrame();
    const auto jet = core::jet_from_thrust(eig(frame.getOrigin()), eig(frame.getBasis().getColumn(0)), t->getThrust(),
                                           t->getPropellerDiameter(), rho);
    if (jet) jets.push_back(*jet);
  }
  return jets;
}

}  // namespace stonefish_net
