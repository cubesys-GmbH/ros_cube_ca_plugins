cmake_minimum_required(VERSION 3.8)
project(custom_ca_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(cube_ca_plugins REQUIRED)
find_package(pluginlib REQUIRED)

add_library(custom_ca_plugins src/custom_dissemination_rules.cpp)
target_compile_features(custom_ca_plugins PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(custom_ca_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(custom_ca_plugins cube_ca_plugins)
pluginlib_export_plugin_description_file(cube_ca_plugins plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS custom_ca_plugins
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
