//
// Created by karisora on 2025/09/12.
//

#ifndef ARES_SENSOR_UART_HPP
#define ARES_SENSOR_UART_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>
#include <string>

class ImuRpyPublisher : public rclcpp::Node {

public:
    ImuRpyPublisher(const std::string & config_path);
private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    std::string imu_topic_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr roll_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pitch_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_pub_;
    void load_config(const std::string & config_path);
};

#endif //ARES_SENSOR_UART_HPP