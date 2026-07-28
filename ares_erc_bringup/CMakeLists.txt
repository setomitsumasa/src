cmake_minimum_required(VERSION 3.8)
project(ares_erc_bringup)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

# --- Phase 1b: map_anchor C++ node (GICP prior-map relocalization) ---
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(map_anchor src/map_anchor.cpp)
target_include_directories(map_anchor PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(map_anchor ${PCL_LIBRARIES})
ament_target_dependencies(map_anchor
  rclcpp sensor_msgs std_msgs geometry_msgs tf2_ros pcl_conversions)

# --- offline .pcd downsampler (keeps RViz/prior-map display from OOMing) ---
add_executable(downsample_map src/downsample_map.cpp)
target_include_directories(downsample_map PUBLIC ${PCL_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIRS})
target_link_libraries(downsample_map ${PCL_LIBRARIES} ${YAML_CPP_LIBRARIES})
ament_target_dependencies(downsample_map ament_index_cpp)

install(TARGETS map_anchor downsample_map DESTINATION lib/${PROJECT_NAME})

# --- ERC launch/config/rviz + the Phase 1a Python node ---
install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/erc_waypoints.py
    scripts/erc_odometry.py
    scripts/erc_cmd_vel_gate.py
    scripts/aruco_detection_adapter.py
    scripts/aruco_map_calibrator.py
    scripts/aruco_map_anchor.py
    scripts/aruco_fake_detections.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_aruco_localization_math
    test/test_aruco_localization_math.py
    TIMEOUT 60
  )
endif()

ament_package()
