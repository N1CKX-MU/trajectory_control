cmake_minimum_required(VERSION 3.14)
project(trajectory_controller VERSION 0.1.0 LANGUAGES CXX)

# ── Compiler settings ──────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wpedantic)

# ── Find ROS2 dependencies ─────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)


# ---- Add executable and link libraries ───────────────────────────────────────
add_executable(waypoints_node src/waypoints.cpp)
ament_target_dependencies(waypoints_node
  rclcpp
  visualization_msgs
  geometry_msgs
)


target_include_directories(waypoints_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(catmullrom_node src/catmull.cpp)
ament_target_dependencies(catmullrom_node
  rclcpp
  visualization_msgs
  geometry_msgs
)

target_include_directories(catmullrom_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)





# ── Install directories ────────────────────────────────────────────────────
install(TARGETS
  waypoints_node
  catmullrom_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  worlds
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# ── Testing ────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
