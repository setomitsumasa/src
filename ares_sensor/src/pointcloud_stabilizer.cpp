//
// Created by karisora on 2025/09/12.
//

#include "ares_sensor/pointcloud_stabilizer.hpp"
#include <cmath>
#include <cstring>

PointCloudStabilizer::PointCloudStabilizer()
: Node("pointcloud_stabilizer"),
  current_roll_(0.0f),
  current_pitch_(0.0f),
  current_yaw_(0.0f),
  roll_received_(false),
  pitch_received_(false),
  yaw_received_(false)
{
    // Subscribers for IMU RPY values
    roll_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "imu/roll", 10,
        std::bind(&PointCloudStabilizer::roll_callback, this, std::placeholders::_1)
    );
    
    pitch_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "imu/pitch", 10,
        std::bind(&PointCloudStabilizer::pitch_callback, this, std::placeholders::_1)
    );
    
    yaw_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "imu/yaw", 10,
        std::bind(&PointCloudStabilizer::yaw_callback, this, std::placeholders::_1)
    );
    
    // Subscriber for point cloud
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/realsense/cloud", 10,
        std::bind(&PointCloudStabilizer::pointcloud_callback, this, std::placeholders::_1)
    );
    
    // Publisher for stabilized point cloud
    stabilized_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/realsense/cloud/stabilized", 10
    );
    
    RCLCPP_INFO(this->get_logger(), "PointCloud Stabilizer node started");
}

void PointCloudStabilizer::roll_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(rpy_mutex_);
    current_roll_ = msg->data;
    roll_received_ = true;
}

void PointCloudStabilizer::pitch_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(rpy_mutex_);
    current_pitch_ = msg->data;
    pitch_received_ = true;
}

void PointCloudStabilizer::yaw_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(rpy_mutex_);
    current_yaw_ = msg->data;
    yaw_received_ = true;
}

void PointCloudStabilizer::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // Check if we have received IMU data
    {
        std::lock_guard<std::mutex> lock(rpy_mutex_);
        if (!roll_received_ || !pitch_received_ || !yaw_received_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "IMU data not yet received, skipping point cloud stabilization"
            );
            return;
        }
    }
    
    float roll, pitch, yaw;
    {
        std::lock_guard<std::mutex> lock(rpy_mutex_);
        roll = current_roll_;
        pitch = current_pitch_;
        yaw = current_yaw_;
    }
    
    // Apply rotation to stabilize the point cloud (compensate for roll and pitch)
    // To keep the point cloud parallel to the ground, we need to rotate by -roll and -pitch
    sensor_msgs::msg::PointCloud2 stabilized_cloud = apply_rotation(*msg, -roll, -pitch, 0.0f);
    
    stabilized_pub_->publish(stabilized_cloud);
}

sensor_msgs::msg::PointCloud2 PointCloudStabilizer::apply_rotation(
    const sensor_msgs::msg::PointCloud2& cloud,
    float roll, float pitch, float yaw
) {
    // Create rotation quaternion from RPY
    tf2::Quaternion rotation_quat;
    rotation_quat.setRPY(static_cast<double>(roll), static_cast<double>(pitch), static_cast<double>(yaw));
    
    // Create rotation matrix
    tf2::Matrix3x3 rotation_matrix(rotation_quat);
    
    // Create output point cloud
    sensor_msgs::msg::PointCloud2 output_cloud = cloud;
    
    // Check if point cloud has x, y, z fields
    bool has_x = false, has_y = false, has_z = false;
    int x_offset = -1, y_offset = -1, z_offset = -1;
    
    for (const auto& field : cloud.fields) {
        if (field.name == "x") {
            has_x = true;
            x_offset = field.offset;
        } else if (field.name == "y") {
            has_y = true;
            y_offset = field.offset;
        } else if (field.name == "z") {
            has_z = true;
            z_offset = field.offset;
        }
    }
    
    if (!has_x || !has_y || !has_z) {
        RCLCPP_ERROR(this->get_logger(), "Point cloud does not have x, y, z fields");
        return cloud;
    }
    
    // Resize output data
    output_cloud.data.resize(cloud.data.size());
    
    // Apply rotation to each point
    for (size_t i = 0; i < cloud.width * cloud.height; ++i) {
        size_t point_offset = i * cloud.point_step;
        
        // Read original point coordinates
        float x, y, z;
        std::memcpy(&x, &cloud.data[point_offset + x_offset], sizeof(float));
        std::memcpy(&y, &cloud.data[point_offset + y_offset], sizeof(float));
        std::memcpy(&z, &cloud.data[point_offset + z_offset], sizeof(float));
        
        // Apply rotation
        tf2::Vector3 point(x, y, z);
        tf2::Vector3 rotated_point = rotation_matrix * point;
        
        // Write rotated point coordinates
        float new_x = static_cast<float>(rotated_point.x());
        float new_y = static_cast<float>(rotated_point.y());
        float new_z = static_cast<float>(rotated_point.z());
        
        // Copy all data from original point
        std::memcpy(&output_cloud.data[point_offset], &cloud.data[point_offset], cloud.point_step);
        
        // Update x, y, z coordinates
        std::memcpy(&output_cloud.data[point_offset + x_offset], &new_x, sizeof(float));
        std::memcpy(&output_cloud.data[point_offset + y_offset], &new_y, sizeof(float));
        std::memcpy(&output_cloud.data[point_offset + z_offset], &new_z, sizeof(float));
    }
    
    return output_cloud;
}

