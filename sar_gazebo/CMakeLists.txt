cmake_minimum_required(VERSION 3.5)
project(sar_gazebo)

# Default settings
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ignition-gazebo3 REQUIRED)

# simple_plugin library
add_library(simple_plugin SHARED src/simple_plugin.cpp)
ament_target_dependencies(simple_plugin ignition-gazebo3)

# ignition-plugin1 setup
set(IGN_PLUGIN_VER 1)
ign_find_package(ignition-plugin1 REQUIRED COMPONENTS register)

# SampleSystem library
add_library(SampleSystem SHARED src/SampleSystem.cc)
set_property(TARGET SampleSystem PROPERTY CXX_STANDARD 17)
target_link_libraries(SampleSystem
  ignition-gazebo3::ignition-gazebo3
  ignition-plugin1::register
)


# Testing setup
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install targets
install(TARGETS simple_plugin SampleSystem
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

ament_package()
