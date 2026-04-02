//
// Created by karisora on 2025/09/10.
//

#include "ares_nav2/gps_waypoint_follower.hpp"
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>
#include <cmath>  // M_PI のために追加

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    std::string yaml = ament_index_cpp::get_package_share_directory("ares_nav2")
                     + std::string("/config/waypoints.yaml");

    ares_nav2::SpiralParams sp;
    sp.r0 = 1.0; sp.dr = 1.0; sp.dtheta_rad = M_PI / 2.0; sp.n_spiral = 10;

    auto node = std::make_shared<ares_nav2::GPSWaypointFollower>(yaml, sp);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
