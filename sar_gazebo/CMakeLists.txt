cmake_minimum_required(VERSION 3.5)
project(sar_gazebo)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ignition-gazebo3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ignition-physics3 REQUIRED)
find_package(ignition-math6 REQUIRED)

include_directories(
  plugins/include
  ${ignition-gazebo3_INCLUDE_DIRS}
  ${ignition-physics3_INCLUDE_DIRS}
  ${ignition-math6_INCLUDE_DIRS}
)

add_library(MyPlugin SHARED plugins/src/MyPlugin.cpp)
ament_target_dependencies(MyPlugin rclcpp)
target_link_libraries(MyPlugin 
  ignition-gazebo3::ignition-gazebo3
  ignition-physics3::ignition-physics3
  ignition-math6::ignition-math6
)

install(TARGETS MyPlugin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)


add_library(Motor_Plugin SHARED plugins/src/Motor_Plugin.cpp)
ament_target_dependencies(Motor_Plugin rclcpp)
target_link_libraries(Motor_Plugin 
  ignition-gazebo3::ignition-gazebo3
  ignition-physics3::ignition-physics3
  ignition-math6::ignition-math6
)

install(TARGETS Motor_Plugin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)


add_library(MyStstemPlugin SHARED plugins/src/MyStstemPlugin.cpp)
ament_target_dependencies(MyStstemPlugin rclcpp)
target_link_libraries(MyStstemPlugin 
  ignition-gazebo3::ignition-gazebo3
  ignition-physics3::ignition-physics3
  ignition-math6::ignition-math6
)

install(TARGETS MyStstemPlugin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

add_library(MyJointControllerPlugin SHARED plugins/src/MyJointControllerPlugin.cpp)
ament_target_dependencies(MyJointControllerPlugin rclcpp)
target_link_libraries(MyJointControllerPlugin 
  ignition-gazebo3::ignition-gazebo3
  ignition-physics3::ignition-physics3
  ignition-math6::ignition-math6
)

install(TARGETS MyJointControllerPlugin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

add_library(DiffDrive SHARED plugins/src/DiffDrive.cc)
ament_target_dependencies(DiffDrive rclcpp)
target_link_libraries(DiffDrive 
  ignition-gazebo3::ignition-gazebo3
  ignition-physics3::ignition-physics3
  ignition-math6::ignition-math6
)

install(TARGETS DiffDrive
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)


#install(DIRECTORY include/
#  DESTINATION include)

#install(DIRECTORY src/
#DESTINATION src)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
