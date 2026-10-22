// Simulador de Stonefish con la red: copia de stonefish_ros2/src/stonefish_simulator.cpp que usa
// NetSimulationManager y recibe la ruta de config/net.yaml como séptimo argumento.
#include <csignal>
#include <cstdlib>
#include <execinfo.h>
#include <fcntl.h>
#include <iostream>
#include <unistd.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "rclcpp/rclcpp.hpp"
#include "stonefish_ros2/ROS2GraphicalSimulationApp.h"
#include "stonefish_net/NetSimulationManager.h"
#include "stonefish_net/core/buoy_asset.hpp"
#include "stonefish_net/core/net_config.hpp"
#include "stonefish_net/core/net_topology.hpp"

using namespace std::chrono_literals;

namespace
{
// Todo lo que el simulador necesita de la red, cargado y validado antes de abrir la ventana.
struct NetSetup
{
  stonefish_net::core::NetConfig cfg;
  stonefish_net::core::NetTopology topo;
  stonefish_net::core::BuoyAsset buoy;
};

class StonefishNetNode : public rclcpp::Node
{
public:
  StonefishNetNode(const std::string& scenarioPath, const std::string& dataPath, NetSetup setup,
                   const sf::RenderSettings& s, const sf::HelperSettings& h, sf::Scalar rate)
  : Node("stonefish_simulator")
  {
    // El nodo ya es propiedad del shared_ptr de main(): el manager recibe un alias sin propiedad
    // (deleter vacío) para no borrarlo dos veces. stonefish_simulator lo copia con un shared_ptr(this).
    auto* manager = new stonefish_net::NetSimulationManager(rate, scenarioPath, std::move(setup.cfg),
                                                            std::move(setup.topo), std::move(setup.buoy),
                                                            std::shared_ptr<rclcpp::Node>(this, [](auto*) {}));
    app_ = std::shared_ptr<sf::ROS2GraphicalSimulationApp>(
      new sf::ROS2GraphicalSimulationApp("Stonefish Simulator (red)", dataPath, s, h, manager));
    app_->Startup();
    tickTimer_ = this->create_wall_timer(16667us, std::bind(&sf::ROS2GraphicalSimulationApp::Tick, app_));
  }

private:
  std::shared_ptr<sf::ROS2GraphicalSimulationApp> app_;
  rclcpp::TimerBase::SharedPtr tickTimer_;
};

// low | medium | high | noshadows (calidad media sin sombras: los hilos de 2,5 mm parpadean en el mapa de sombras)
sf::RenderSettings render_settings(const std::string& quality, int width, int height)
{
  const bool low = quality == "low";
  const sf::RenderQuality q = low ? sf::RenderQuality::LOW
                            : quality == "high" ? sf::RenderQuality::HIGH : sf::RenderQuality::MEDIUM;
  sf::RenderSettings s;
  s.windowW = width;
  s.windowH = height;
  s.shadows = quality == "noshadows" ? sf::RenderQuality::DISABLED : q;
  s.ao = low ? sf::RenderQuality::DISABLED : q;
  s.atmosphere = q;
  s.ocean = q;
  s.aa = q;
  s.ssr = low ? sf::RenderQuality::DISABLED : q;
  return s;
}

sf::HelperSettings helper_settings()
{
  sf::HelperSettings h;
  h.showFluidDynamics = false;
  h.showCoordSys = false;
  h.showBulletDebugInfo = false;
  h.showSensors = false;
  h.showActuators = false;
  h.showForces = false;
  return h;
}

int g_crash_fd = -1;  // ~/.ros/log/stonefish_net_crash.log, abierto al arrancar (ros2 launch no guarda stderr)

// Si el simulador se cae, deja la pila en stderr y en g_crash_fd (en humble-gpu no hay gdb).
// Async-signal-safe: solo write y backtrace_symbols_fd.
void print_backtrace(int sig)
{
  void* frames[64];
  const int n = backtrace(frames, 64);
  const char msg[] = "\n*** stonefish_net_simulator: señal fatal, pila:\n";
  for (int fd : {STDERR_FILENO, g_crash_fd}) {
    if (fd < 0) continue;
    [[maybe_unused]] auto ignored = write(fd, msg, sizeof(msg) - 1);
    backtrace_symbols_fd(frames, n, fd);
  }
  std::signal(sig, SIG_DFL);
  std::raise(sig);
}
}  // namespace

int main(int argc, char** argv)
{
  // La primera llamada a backtrace() carga libgcc_s y reserva memoria: se hace ahora, para que el manejador
  // funcione también cuando la señal llega con el montón corrupto ("double free or corruption").
  void* warmup[1];
  backtrace(warmup, 1);
  if (const char* home = std::getenv("HOME")) {
    g_crash_fd = open((std::string(home) + "/.ros/log/stonefish_net_crash.log").c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
  }
  std::signal(SIGSEGV, print_backtrace);
  std::signal(SIGABRT, print_backtrace);
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  if (argc < 8) {
    std::cerr << "Uso: stonefish_net_simulator <data> <escenario> <rate> <ancho> <alto> <calidad> <net.yaml> "
                 "[semilla_daño]"
              << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  const std::string dataPath = std::string(argv[1]) + "/";
  const std::string scenarioPath(argv[2]);
  const sf::Scalar rate = atof(argv[3]);
  const std::string netConfigPath(argv[7]);
  const int damageSeed = argc > 8 ? atoi(argv[8]) : -1;  // < 0: la del YAML

  // Un YAML o un asset erróneos salen con 1 y un mensaje, no a mitad de arranque
  NetSetup setup;
  try {
    setup.cfg = stonefish_net::core::load_net_config(netConfigPath);
    if (damageSeed >= 0) {
      setup.cfg.damage.seed = damageSeed;
    }
    setup.topo = stonefish_net::core::build_topology(setup.cfg);
    setup.buoy = stonefish_net::core::BuoyAsset::load(
      ament_index_cpp::get_package_share_directory("stonefish_net") + "/data/buoy");
  } catch (const std::exception& e) {
    std::cerr << "stonefish_net_simulator: configuración de la red '" << netConfigPath << "' no válida: " << e.what()
              << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  std::shared_ptr<StonefishNetNode> node(new StonefishNetNode(scenarioPath, dataPath, std::move(setup),
                                                              render_settings(argv[6], atoi(argv[4]), atoi(argv[5])),
                                                              helper_settings(), rate));
  rclcpp::spin(node);
  return 0;
}
