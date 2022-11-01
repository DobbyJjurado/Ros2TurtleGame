cmake_minimum_required(VERSION 3.5)
project(turtle_game)

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

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(turtle_game_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_executable(turtle_spawner src/turtle_spawner.cpp)
ament_target_dependencies(turtle_spawner rclcpp turtlesim turtle_game_interfaces)

add_executable(turtle_logic src/turtle_logic.cpp)
ament_target_dependencies(turtle_logic rclcpp turtlesim turtle_game_interfaces geometry_msgs)

install(TARGETS
  turtle_spawner
  turtle_logic
  DESTINATION lib/${PROJECT_NAME})

ament_package()
