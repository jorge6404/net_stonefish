cmake_minimum_required(VERSION 3.8)
project(stonefish_net)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# --- net_core: topología e hidrodinámica, sin ROS ni Stonefish ---
add_library(net_core STATIC
  src/core/net_config.cpp
  src/core/net_topology.cpp
  src/core/topology/lattice.cpp
  src/core/topology/rigging.cpp
  src/core/topology/damage.cpp
  src/core/topology/graph.cpp
  src/core/hydro.cpp
  src/core/contact.cpp
  src/core/tiles.cpp
  src/core/coloring.cpp
  src/core/gpu_layout.cpp
  src/core/xpbd.cpp
  src/core/xpbd_ref.cpp
  src/core/current_grid.cpp
  src/core/realtime.cpp
  src/core/grasp.cpp
  src/core/buoy_asset.cpp
)
target_include_directories(net_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(net_core PUBLIC Eigen3::Eigen yaml-cpp)

# --- adaptador a Stonefish (librería privada del ejecutable) y simulador ---
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(stonefish_ros2 REQUIRED)
# dependencias que las cabeceras de stonefish_ros2 incluyen pero el paquete no exporta
set(STONEFISH_ROS2_HEADER_DEPS geometry_msgs nav_msgs sensor_msgs std_msgs std_srvs image_transport tf2_ros)
foreach(dep ${STONEFISH_ROS2_HEADER_DEPS})
  find_package(${dep} REQUIRED)
endforeach()
find_package(Stonefish REQUIRED 1.6.0)

# --- net_gpu: contexto de OpenGL y solver XPBD en compute shaders; sin ROS (solo el glad y el SDL2 de Stonefish) ---
add_library(net_gpu STATIC
  src/gl/GlComputeContext.cpp
  src/gpu/GpuXpbd.cpp
)
target_include_directories(net_gpu PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(net_gpu PUBLIC net_core Stonefish::Stonefish)

add_library(stonefish_net_adapter STATIC
  src/NetEntity.cpp
  src/NetForces.cpp
  src/BulletSolver.cpp
  src/GpuSolver.cpp
  src/RobotContact.cpp
  src/SolidContacts.cpp
  src/NetSleep.cpp
  src/NetSimulationManager.cpp
  src/diagnostics.cpp
)
target_include_directories(stonefish_net_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
# stonefish_ros2 instala sus cabeceras en include/include/ pero exporta include/: se añade la ruta real
get_filename_component(STONEFISH_ROS2_PREFIX "${stonefish_ros2_DIR}/../../.." ABSOLUTE)
target_include_directories(stonefish_net_adapter PUBLIC ${STONEFISH_ROS2_PREFIX}/include/include)
find_library(STONEFISH_ROS2_INTERFACE_LIB stonefish_ros2_interface HINTS ${STONEFISH_ROS2_PREFIX}/lib REQUIRED)
# Solo la Bullet que trae Stonefish (Stonefish::Stonefish), nunca /usr/include/bullet
target_link_libraries(stonefish_net_adapter net_core net_gpu Stonefish::Stonefish ${STONEFISH_ROS2_INTERFACE_LIB})
ament_target_dependencies(stonefish_net_adapter rclcpp ament_index_cpp stonefish_ros2 ${STONEFISH_ROS2_HEADER_DEPS})

add_executable(stonefish_net_simulator src/stonefish_net_simulator.cpp)
target_link_libraries(stonefish_net_simulator stonefish_net_adapter)
target_link_options(stonefish_net_simulator PRIVATE -rdynamic)  # nombres en la pila del manejador de señales
ament_target_dependencies(stonefish_net_simulator rclcpp ament_index_cpp stonefish_ros2)

# Escenas de prueba comunes a los tests y a gpu_check, con la red fija de test/net_test.yaml
add_library(net_test_scene STATIC test/support/scene.cpp)
target_include_directories(net_test_scene PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/test)
target_compile_definitions(net_test_scene PRIVATE NET_TEST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/test/net_test.yaml")
target_link_libraries(net_test_scene PUBLIC net_core)

# Comprobaciones del solver en la GPU (tools/gpu_check.cpp): se ejecuta a mano desde build/, no se instala
add_executable(gpu_check tools/gpu_check.cpp)
target_link_libraries(gpu_check net_gpu net_test_scene)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Las de la red completa con el XPBD de referencia tardan minutos
  foreach(name test_config test_topology test_realtime test_xpbd test_grasp)
    ament_add_gtest(${name} test/${name}.cpp TIMEOUT 900)
    target_link_libraries(${name} net_test_scene)
  endforeach()
endif()

install(TARGETS stonefish_net_simulator DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config data launch scenarios shaders DESTINATION share/${PROJECT_NAME})

ament_package()
