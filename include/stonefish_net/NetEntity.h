// El dibujo de la red dentro de Stonefish: tubos por los nudos que da el solver activo (NetSolver::positions).
// La física no está aquí: BulletSolver o el solver de GPU.
//
// Hereda de ForcefieldEntity a propósito: Stonefish castea las entidades según su EntityType
// (CABLE → CableEntity*, SOLID → SolidEntity*, ...). El único trato a un FORCEFIELD en el tick
// es comprobar getForcefieldType() == TRIGGER (virtual), así que es el tipo seguro para una
// entidad propia. No se añade el ghost de la base al mundo: AddToSimulation no hace nada.
#pragma once

#include <string>
#include <vector>

#include <Stonefish/entities/ForcefieldEntity.h>
#include <Stonefish/graphics/OpenGLDataStructs.h>

#include "stonefish_net/NetSolver.h"
#include "stonefish_net/core/net_config.hpp"
#include "stonefish_net/core/net_topology.hpp"

namespace sf
{
class SimulationManager;
}

namespace stonefish_net
{

class NetEntity : public sf::ForcefieldEntity
{
public:
  NetEntity(std::string name, const core::NetTopology& topo, const core::NetConfig& cfg);

  sf::ForcefieldType getForcefieldType() override { return sf::ForcefieldType::POOL; }
  void AddToSimulation(sf::SimulationManager*) override {}
  std::vector<sf::Renderable> Render() override;
  void getAABB(sf::Vector3& min, sf::Vector3& max) override;

  // El solver cuyas posiciones se dibujan; sin él no se dibuja nada.
  void set_solver(const NetSolver* solver) { solver_ = solver; }

private:
  // Un tubo dibujado: hilo continuo (sus nudos) o plomo (un nudo, dibujado como cilindro corto).
  // `item` se reserva una vez y Render() solo reescribe sus CableNode.
  struct CableView
  {
    std::vector<int> nodes;
    float radius;
    sf::Renderable item;
  };

  core::NetTopology topo_;
  core::NetConfig cfg_;
  const NetSolver* solver_ = nullptr;
  std::vector<CableView> strands_;
  std::vector<CableView> leads_;
};

}  // namespace stonefish_net
