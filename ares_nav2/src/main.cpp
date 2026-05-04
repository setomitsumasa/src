//
// Created by karisora on 2025/09/10.
//

#include "ares_nav2/gps_waypoint_follower.hpp"
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>
#include <vector>
#include <cmath>  // M_PI のために追加

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    const std::string package_share = ament_index_cpp::get_package_share_directory("ares_nav2");
    std::string yaml = package_share + std::string("/config/waypoints.yaml");

    const auto non_ros_args = rclcpp::remove_ros_arguments(argc, argv);
    for (size_t i = 1; i < non_ros_args.size(); ++i) {
        if (non_ros_args[i] == "--waypoints-file" && i + 1 < non_ros_args.size()) {
            yaml = non_ros_args[i + 1];
            ++i;
        }
    }

    ares_nav2::SpiralParams sp;
    sp.r0 = 1.0; sp.dr = 1.0; sp.dtheta_rad = M_PI / 2.0; sp.n_spiral = 10;

    auto node = std::make_shared<ares_nav2::GPSWaypointFollower>(yaml, sp);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
