cmake_minimum_required(VERSION 3.8)
project(cube_ca_plugins)

set(CMAKE_CXX_VISIBILITY_PRESET "hidden")
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(etsi_its_cam_msgs REQUIRED)
find_package(rclcpp REQUIRED)

pluginlib_export_plugin_description_file(cube_ca_plugins plugins.xml)

add_library(cube_ca_plugins
  src/cam_utils.cpp
  src/rsu_dissemination_rules.cpp
  src/vehicle_dissemination_rules.cpp)
add_library(cube_ca_plugins::cube_ca_plugins ALIAS cube_ca_plugins)
target_compile_features(cube_ca_plugins PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(cube_ca_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(
  cube_ca_plugins
  etsi_its_cam_msgs
  pluginlib
  rclcpp
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(cube_ca_plugins PRIVATE "CUBE_CA_PLUGINS_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS cube_ca_plugins
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)
ament_export_libraries(
  cube_ca_plugins
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
