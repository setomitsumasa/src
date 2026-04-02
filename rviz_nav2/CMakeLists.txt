cmake_minimum_required(VERSION 3.8)
project(rviz_nav2)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

set(
        headers_to_moc
        include/rviz_nav2/aruco_id_display.hpp
)

foreach (header "${headers_to_moc}")
    qt5_wrap_cpp(display_moc_files "${header}")
endforeach ()

set(
        display_source_files
        src/aruco_id_display.cpp
)

add_library(
        ${PROJECT_NAME} SHARED
        ${display_moc_files}
        ${display_source_files}
)

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)

target_include_directories(
        ${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${Qt5Widgets_INCLUDE_DIRS}
)

target_link_libraries(
        ${PROJECT_NAME} PUBLIC
        rclcpp::rclcpp
        rviz_common::rviz_common
        ${std_msgs_TARGETS}
        Qt5::Core
        Qt5::Widgets
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "${PROJECT_NAME}_BUILDING_LIBRARY")

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
        rclcpp
        rviz_common
        std_msgs
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

install(
        DIRECTORY include/
        DESTINATION include
)

install(
        FILES plugins_description.xml
        DESTINATION share/${PROJECT_NAME}
)

ament_package()

