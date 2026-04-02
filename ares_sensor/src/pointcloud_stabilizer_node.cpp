//
// Created by karisora on 2025/09/12.
//

#include "ares_sensor/pointcloud_stabilizer.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<PointCloudStabilizer>();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

