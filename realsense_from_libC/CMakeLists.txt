cmake_minimum_required(VERSION 3.8)
project(realsense_from_libC)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(realsense2 REQUIRED)

add_executable(publish_realsense
  src/publish_realsense.cpp
)
target_include_directories(publish_realsense
  PRIVATE
    ${realsense2_INCLUDE_DIR}
)
target_link_libraries(publish_realsense
  ${realsense2_LIBRARY}
)
ament_target_dependencies(publish_realsense
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS
  publish_realsense
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
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
