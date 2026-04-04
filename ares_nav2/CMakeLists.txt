cmake_minimum_required(VERSION 3.14)
project(ares_nav2)

# Warnings and C++ standard
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

# Library
add_library(ares_nav2
        src/gps_waypoint_follower.cpp
        src/geo_utils.cpp
        src/waypoint_loader.cpp
)
target_compile_features(ares_nav2 PUBLIC cxx_std_17)
target_include_directories(ares_nav2 PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(ares_nav2
        rclcpp
        rclcpp_action
        geometry_msgs
        nav2_msgs
        sensor_msgs
        ament_index_cpp
        std_msgs
        tf2_ros
)
target_link_libraries(ares_nav2 yaml-cpp)

add_executable(gps_waypoint_follower_node
        src/main.cpp
)

target_compile_features(gps_waypoint_follower_node PUBLIC cxx_std_17)
target_link_libraries(gps_waypoint_follower_node PRIVATE ares_nav2)

# ArUco marker TF -> Nav2 goal
add_executable(aruco_nav2_goal_node
  src/aruco_nav2_goal_main.cpp
  src/aruco_nav2_goal.cpp
)
target_include_directories(aruco_nav2_goal_node PRIVATE include)
target_compile_features(aruco_nav2_goal_node PUBLIC cxx_std_17)
ament_target_dependencies(aruco_nav2_goal_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  rclcpp_components
)

add_executable(yolo_tf_nav2_goal_node
  src/yolo_tf_nav2_goal_main.cpp
  src/yolo_tf_nav2_goal.cpp
)
target_include_directories(yolo_tf_nav2_goal_node PRIVATE include)
target_compile_features(yolo_tf_nav2_goal_node PUBLIC cxx_std_17)
ament_target_dependencies(yolo_tf_nav2_goal_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  std_msgs
  tf2_ros
)


# Custom BT nodes library
add_library(ares_nav2_bt_nodes SHARED
        src/bt_plugins/custom_rover_recovery.cpp
)
ament_target_dependencies(ares_nav2_bt_nodes
  rclcpp
  geometry_msgs
  behaviortree_cpp_v3
)
target_include_directories(ares_nav2_bt_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Tests
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

# Install
install(TARGETS
        ares_nav2
        gps_waypoint_follower_node
        aruco_nav2_goal_node
        yolo_tf_nav2_goal_node
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS ares_nav2_bt_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config OPTIONAL)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})


# Export
ament_export_include_directories(include)
ament_export_dependencies(
        rclcpp
        rclcpp_action
        geometry_msgs
        nav2_msgs
        sensor_msgs
        ament_index_cpp
)

ament_package()
