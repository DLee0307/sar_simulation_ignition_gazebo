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

include_directories(plugins/include ${ignition-gazebo3_INCLUDE_DIRS})

add_library(MyPlugin SHARED plugins/src/MyPlugin.cpp)
target_link_libraries(MyPlugin ignition-gazebo3::ignition-gazebo3)

install(TARGETS MyPlugin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

add_library(Motor_Plugin SHARED plugins/src/Motor_Plugin.cpp)
target_link_libraries(Motor_Plugin ignition-gazebo3::ignition-gazebo3)

install(TARGETS Motor_Plugin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

#install(DIRECTORY include/
#  DESTINATION include)

#install(DIRECTORY src/
#DESTINATION src)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
