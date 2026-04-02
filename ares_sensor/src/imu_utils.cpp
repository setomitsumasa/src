//
// Created by karisora on 2025/09/12.
//

#include "ares_sensor/imu_utils.hpp"
#include <yaml-cpp/yaml.h>
#include <tf2/LinearMath/Matrix3x3.h>

ImuRpyPublisher::ImuRpyPublisher(const std::string & config_path)
: Node("imu_rpy_publisher")
{
    load_config(config_path);

    roll_pub_ = this->create_publisher<std_msgs::msg::Float32>("imu/roll", 10);
    pitch_pub_ = this->create_publisher<std_msgs::msg::Float32>("imu/pitch", 10);
    yaw_pub_ = this->create_publisher<std_msgs::msg::Float32>("imu/yaw", 10);

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, 10,
        std::bind(&ImuRpyPublisher::imu_callback, this, std::placeholders::_1)
    );
}

void ImuRpyPublisher::load_config(const std::string & config_path) {
    YAML::Node config = YAML::LoadFile(config_path);
    imu_topic_ = config["sensor"]["imu"]["topic"].as<std::string>();
}

void ImuRpyPublisher::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    const auto & q = msg->orientation;
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    auto roll_msg = std_msgs::msg::Float32();
    auto pitch_msg = std_msgs::msg::Float32();
    auto yaw_msg = std_msgs::msg::Float32();
    roll_msg.data = static_cast<float>(roll);
    pitch_msg.data =static_cast<float>(pitch);
    yaw_msg.data = static_cast<float>(yaw);

    roll_pub_->publish(roll_msg);
    pitch_pub_->publish(pitch_msg);
    yaw_pub_->publish(yaw_msg);
}
