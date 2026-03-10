```cmake
cmake_minimum_required(VERSION 3.14)
project(trajectory_controller VERSION 0.1.0 LANGUAGES CXX)

# ── Compiler settings ─────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wpedantic)

# ── Find ROS2 dependencies ────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# ── Common include directory ──────────────────────────────────────
set(PROJECT_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

# ── Node: Waypoints ───────────────────────────────────────────────
add_executable(waypoints_node src/waypoints.cpp)
ament_target_dependencies(waypoints_node
  rclcpp visualization_msgs geometry_msgs
)
target_include_directories(waypoints_node PUBLIC ${PROJECT_INCLUDE_DIR})

# ── Node: Catmull-Rom ─────────────────────────────────────────────
add_executable(catmullrom_node src/catmull.cpp)
ament_target_dependencies(catmullrom_node
  rclcpp visualization_msgs geometry_msgs
)
target_include_directories(catmullrom_node PUBLIC ${PROJECT_INCLUDE_DIR})

# ── Node: Bezier ──────────────────────────────────────────────────
add_executable(bezier_node src/bezier_node.cpp)
ament_target_dependencies(bezier_node
  rclcpp visualization_msgs geometry_msgs
)
target_include_directories(bezier_node PUBLIC ${PROJECT_INCLUDE_DIR})

# ── Node: Gradient Descent Smoothing ──────────────────────────────
add_executable(gradient_descent_node src/gradient_smoothing.cpp)
ament_target_dependencies(gradient_descent_node
  rclcpp visualization_msgs geometry_msgs
)
target_include_directories(gradient_descent_node PUBLIC ${PROJECT_INCLUDE_DIR})

# ── Node: Obstacle Avoider ────────────────────────────────────────
add_executable(obstacle_avoider_node src/obstacle_avoider.cpp)
ament_target_dependencies(obstacle_avoider_node
  rclcpp visualization_msgs geometry_msgs
)
target_include_directories(obstacle_avoider_node PUBLIC ${PROJECT_INCLUDE_DIR})

# ── Node: Trajectory Generator ────────────────────────────────────
add_executable(trajectory_generator_node src/trajectory_generator.cpp)
ament_target_dependencies(trajectory_generator_node
  rclcpp nav_msgs geometry_msgs visualization_msgs std_msgs
  tf2 tf2_ros tf2_geometry_msgs
)
target_include_directories(trajectory_generator_node PUBLIC ${PROJECT_INCLUDE_DIR})

# ── Node: Controller ──────────────────────────────────────────────
add_executable(controller_node src/controller.cpp)
ament_target_dependencies(controller_node
  rclcpp geometry_msgs nav_msgs std_msgs
  tf2 tf2_ros tf2_geometry_msgs visualization_msgs
)
target_include_directories(controller_node PUBLIC ${PROJECT_INCLUDE_DIR})

# ── Node: Obstacle Detector ───────────────────────────────────────
add_executable(obstacle_detector_node src/obstacle_detector.cpp)
ament_target_dependencies(obstacle_detector_node
  rclcpp sensor_msgs visualization_msgs geometry_msgs
  tf2 tf2_ros tf2_geometry_msgs
)
target_include_directories(obstacle_detector_node PUBLIC ${PROJECT_INCLUDE_DIR})

# ── Install all executables together ──────────────────────────────
install(TARGETS
  waypoints_node
  catmullrom_node
  bezier_node
  gradient_descent_node
  obstacle_avoider_node
  trajectory_generator_node
  controller_node
  obstacle_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

# ── Install resources ─────────────────────────────────────────────
install(DIRECTORY
  launch
  config
  worlds
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# ── Testing ───────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```
