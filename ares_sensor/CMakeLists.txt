cmake_minimum_required(VERSION 3.14)
project(ares_sensor)

# Warnings and C++ standard
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Library
add_library(ares_sensor
        src/imu.cpp
        src/imu_utils.cpp
        src/gps.cpp
)
target_compile_features(ares_sensor PUBLIC cxx_std_17)
target_include_directories(ares_sensor PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(ares_sensor
        rclcpp
        rclcpp_action
        geometry_msgs
        sensor_msgs
        ament_index_cpp
        std_msgs
        yaml-cpp
        tf2
        tf2_geometry_msgs
)
target_link_libraries(ares_sensor yaml-cpp)

add_executable(sensor_node
        src/main.cpp
)

target_compile_features(sensor_node PUBLIC cxx_std_17)
target_link_libraries(sensor_node PRIVATE ares_sensor)

add_executable(imu_node
        src/imu.cpp
)

target_compile_features(imu_node PUBLIC cxx_std_17)
target_link_libraries(imu_node PRIVATE ares_sensor)

add_executable(gps_node
        src/gps.cpp
)

target_compile_features(gps_node PUBLIC cxx_std_17)
target_link_libraries(gps_node PRIVATE ares_sensor)

add_executable(sensor_tf_node
        src/sensor_tf.cpp
)

target_compile_features(sensor_tf_node PUBLIC cxx_std_17)
ament_target_dependencies(sensor_tf_node
        rclcpp
        geometry_msgs
        std_msgs
        tf2_ros
)

add_executable(pointcloud_stabilizer_node
        src/pointcloud_stabilizer_node.cpp
        src/pointcloud_stabilizer.cpp
)

target_compile_features(pointcloud_stabilizer_node PUBLIC cxx_std_17)
target_include_directories(pointcloud_stabilizer_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(pointcloud_stabilizer_node
        rclcpp
        sensor_msgs
        std_msgs
        tf2
        tf2_geometry_msgs
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
        ares_sensor
        sensor_node
        imu_node
        gps_node
        sensor_tf_node
        pointcloud_stabilizer_node
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
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
        sensor_msgs
        ament_index_cpp
        std_msgs
        yaml-cpp
        tf2
        tf2_ros
        tf2_geometry_msgs
)

ament_package()
