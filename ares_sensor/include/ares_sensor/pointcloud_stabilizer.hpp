//
// Created by karisora on 2025/09/12.
//

#ifndef ARES_SENSOR_POINTCLOUD_STABILIZER_HPP
#define ARES_SENSOR_POINTCLOUD_STABILIZER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <mutex>

class PointCloudStabilizer : public rclcpp::Node {
public:
    PointCloudStabilizer();

private:
    void roll_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void pitch_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void yaw_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    
    sensor_msgs::msg::PointCloud2 apply_rotation(
        const sensor_msgs::msg::PointCloud2& cloud,
        float roll, float pitch, float yaw
    );

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr roll_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pitch_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr yaw_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr stabilized_pub_;

    float current_roll_;
    float current_pitch_;
    float current_yaw_;
    bool roll_received_;
    bool pitch_received_;
    bool yaw_received_;
    
    std::mutex rpy_mutex_;
};

#endif //ARES_SENSOR_POINTCLOUD_STABILIZER_HPP

