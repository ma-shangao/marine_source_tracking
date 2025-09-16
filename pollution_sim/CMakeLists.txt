cmake_minimum_required(VERSION 3.8)
project(pollution_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# GoogleTest requires at least C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(pollution_interfaces REQUIRED)

include_directories(
  include
)

# Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/pollution_sim.cpp
# )

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  nav_msgs
  grid_map_ros
  grid_map_msgs
  Eigen3
)

# add_executable(${PROJECT_NAME}_node
#   src/pollution_sim_node.cpp
# )
add_executable(${PROJECT_NAME}_node_csv
  src/pollution_sim_csv.cpp
)

# ament_target_dependencies(${PROJECT_NAME}_node
#   rclcpp
# )

ament_target_dependencies(${PROJECT_NAME}_node_csv
  rclcpp
  grid_map_ros
  grid_map_msgs
  nav_msgs
  pollution_interfaces
)

# target_link_libraries(${PROJECT_NAME}_node
#   ${PROJECT_NAME}
# )

install(TARGETS
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}_node_csv
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test test/test_pollution_sim.cpp)
  ament_target_dependencies(${PROJECT_NAME}_test
    rclcpp
  )
  target_link_libraries(${PROJECT_NAME}_test
    ${PROJECT_NAME}
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
