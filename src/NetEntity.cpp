#include "stonefish_net/NetEntity.h"

#include <Stonefish/core/GraphicalSimulationApp.h>
#include <Stonefish/core/SimulationApp.h>
#include <Stonefish/graphics/OpenGLContent.h>
#include <Stonefish/graphics/OpenGLPipeline.h>

namespace stonefish_net
{
namespace
{
constexpr float kLeadRadius = 0.015f;      // plomo dibujado: cilindro Ø 3 cm
constexpr float kLeadHalfLength = 0.03f;   // 6 cm a lo largo de la relinga

struct Style
{
  const char* look;
  float radius;
};

Style style_for(core::LinkKind kind, const core::NetConfig& c)
{
  if (kind == core::LinkKind::Twine) {
    return {"net_twine", static_cast<float>(c.net.twine_diameter / 2.0)};
  }
  return {"net_rope", static_cast<float>(c.frame_rope.diameter / 2.0)};  // marco, orinques y fondeo
}

glm::vec3 glv(const Eigen::Vector3f& v) { return glm::vec3(v.x(), v.y(), v.z()); }

// Escribe un tubo por los puntos dados: tangente por diferencias centradas y coordenada de textura
// a lo largo del tubo en diámetros.
void write_polyline(sf::Renderable& item, const std::vector<glm::vec3>& pts, float radius)
{
  auto nodes = item.getDataAsCableNodes();
  float s = 0.f;
  for (size_t i = 0; i < pts.size(); ++i) {
    if (i > 0) {
      s += glm::length(pts[i] - pts[i - 1]);
    }
    const glm::vec3 t = pts[i + 1 < pts.size() ? i + 1 : i] - pts[i > 0 ? i - 1 : i];
    nodes->at(i).posCoord = glm::vec4(pts[i], s / (2.f * radius));
    nodes->at(i).localTangent = glm::length(t) > 0.f ? glm::normalize(t) : glm::vec3(0, 0, 1);
  }
}
}  // namespace

NetEntity::NetEntity(std::string name, const core::NetTopology& topo, const core::NetConfig& cfg)
: sf::ForcefieldEntity(name), topo_(topo), cfg_(cfg)
{
  // Tubos dibujados: un objeto de cable de Stonefish por hilo continuo y por plomo, reservados una vez
  sf::OpenGLContent* content = sf::SimulationApp::getApp()->hasGraphics()
    ? static_cast<sf::GraphicalSimulationApp*>(sf::SimulationApp::getApp())->getGLPipeline()->getContent()
    : nullptr;
  auto make_view = [&](std::vector<int> nodes, size_t points, const char* look, float radius) {
    CableView v{std::move(nodes), radius, sf::Renderable()};
    v.item.type = sf::RenderableType::CABLE;
    v.item.lookId = content ? content->getLookId(look) : -1;
    v.item.objectId = content ? static_cast<int>(content->BuildCable(points)) : -1;
    v.item.model = glm::mat4(radius);  // el pipeline de cables lee el radio de model[0][0]
    v.item.data = std::make_shared<std::vector<sf::CableNode>>(points);
    return v;
  };
  for (const auto& s : topo_.strands) {
    const Style st = style_for(s.kind, cfg_);
    strands_.push_back(make_view(s.nodes, s.nodes.size(), st.look, st.radius));
  }
  for (int n : topo_.marks.leads) {
    leads_.push_back(make_view({n}, 2, "net_lead", kLeadRadius));
  }
}

std::vector<sf::Renderable> NetEntity::Render()
{
  std::vector<sf::Renderable> items;
  if (solver_ == nullptr) {
    return items;
  }
  const auto& x = solver_->positions();
  items.reserve(strands_.size() + leads_.size());
  std::vector<glm::vec3> pts;
  for (auto& v : strands_) {
    pts.clear();
    for (int n : v.nodes) pts.push_back(glv(x[static_cast<size_t>(n)]));
    write_polyline(v.item, pts, v.radius);
    items.push_back(v.item);
  }
  for (auto& v : leads_) {
    const glm::vec3 c = glv(x[static_cast<size_t>(v.nodes.front())]);
    const glm::vec3 half(0.f, kLeadHalfLength, 0.f);
    write_polyline(v.item, {c - half, c + half}, v.radius);
    items.push_back(v.item);
  }
  return items;
}

void NetEntity::getAABB(sf::Vector3& min, sf::Vector3& max)
{
  Eigen::Vector3f lo = Eigen::Vector3f::Zero(), hi = Eigen::Vector3f::Zero();
  if (solver_ != nullptr && !solver_->positions().empty()) {
    lo = hi = solver_->positions().front();
    for (const auto& p : solver_->positions()) {
      lo = lo.cwiseMin(p);
      hi = hi.cwiseMax(p);
    }
  }
  min = sf::Vector3(lo.x(), lo.y(), lo.z());
  max = sf::Vector3(hi.x(), hi.y(), hi.z());
}

}  // namespace stonefish_net
