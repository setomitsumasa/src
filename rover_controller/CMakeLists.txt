cmake_minimum_required(VERSION 3.14)
project(rover_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(rover_controller_node
  src/rover_controller_node.cpp
)
target_compile_features(rover_controller_node PUBLIC cxx_std_17)
ament_target_dependencies(rover_controller_node
  rclcpp
  geometry_msgs
  std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  rover_controller_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME} OPTIONAL)

ament_package()
