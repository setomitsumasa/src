//
// Created by karisora on 2025/09/12.
//

#include "ares_sensor/imu_utils.hpp"
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    std::string config_path = ament_index_cpp::get_package_share_directory("ares_sensor") + "/config/config.yaml";

    auto node = std::make_shared<ImuRpyPublisher>(config_path);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
