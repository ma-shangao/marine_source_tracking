cmake_minimum_required(VERSION 3.8)
project(pollution_ipp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# specify the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
# Libigl
include(libigl)
igl_include(glfw)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(grid_map REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(pollution_interfaces REQUIRED)
find_package(ardupilot_msgs REQUIRED)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)
find_package(OpenMP REQUIRED)

include_directories(
  src
)

add_library(source_loc
  src/source_loc.cpp
)

add_executable(source_loc_node
  src/source_loc_node.cpp
)

target_link_libraries(source_loc
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  OpenMP::OpenMP_CXX
  igl::glfw
  ${GeographicLib_LIBRARIES}
  ${geometry_msgs_TARGETS}
  ${pollution_interfaces_TARGETS}
  ${ardupilot_msgs_TARGETS}
)
ament_target_dependencies(source_loc
  grid_map_ros
)

target_link_libraries(source_loc_node
  source_loc
)

add_executable(source_loc_client
  src/source_loc_client.cpp
)

target_link_libraries(source_loc_client
  source_loc
)

add_executable(rviz_goal_pose_node
  src/rviz_goal_pose.cpp
)
target_link_libraries(rviz_goal_pose_node
  rclcpp::rclcpp
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  ${geometry_msgs_TARGETS}
  ${GeographicLib_LIBRARIES}
  ${ardupilot_msgs_TARGETS}
)

add_executable(odometry_recorder
  src/odometry_recorder.cpp
)
target_link_libraries(odometry_recorder
  rclcpp::rclcpp
  ${nav_msgs_TARGETS}
)

install(
  TARGETS
    source_loc_node
    # source_loc_server
    source_loc_client
    rviz_goal_pose_node
    odometry_recorder
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(source_loc_test
    test/source_loc_test.cpp
  )
  target_link_libraries(source_loc_test
    source_loc
  )

  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
