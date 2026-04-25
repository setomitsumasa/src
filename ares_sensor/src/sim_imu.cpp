//
// Created by karisora on 2026/04/15.
//

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>

class SimImuNode : public rclcpp::Node
{
public:
    SimImuNode()
    : rclcpp::Node("sim_imu_node")
    {
        using std::placeholders::_1;

        input_topic_ = this->declare_parameter<std::string>("input_topic", "imu");
        output_topic_ = this->declare_parameter<std::string>("output_topic", "imu/data");
        tilt_alpha_ = this->declare_parameter<double>("tilt_alpha", 0.2);
        override_tilt_from_accel_ =
            this->declare_parameter<bool>("override_tilt_from_accel", false);

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            input_topic_,
            rclcpp::QoS(50),
            std::bind(&SimImuNode::imuCallback, this, _1));

        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
            output_topic_,
            rclcpp::QoS(50));

        RCLCPP_INFO(
            this->get_logger(),
            "SimImuNode started. Subscribing %s and publishing %s (override_tilt_from_accel=%s)",
            input_topic_.c_str(),
            output_topic_.c_str(),
            override_tilt_from_accel_ ? "true" : "false");
    }

private:
    bool updateTiltEstimate(
        const sensor_msgs::msg::Imu & msg,
        double & roll_rad,
        double & pitch_rad)
    {
        const double ax = msg.linear_acceleration.x;
        const double ay = msg.linear_acceleration.y;
        const double az = msg.linear_acceleration.z;

        if (!std::isfinite(ax) || !std::isfinite(ay) || !std::isfinite(az)) {
            return tilt_initialized_;
        }

        const double norm = std::sqrt(ax * ax + ay * ay + az * az);
        if (norm < 1e-6) {
            return tilt_initialized_;
        }

        const double nx = ax / norm;
        const double ny = ay / norm;
        const double nz = az / norm;

        const double measured_roll = std::atan2(ny, nz);
        const double measured_pitch = std::atan2(-nx, std::sqrt(ny * ny + nz * nz));

        if (!tilt_initialized_) {
            filtered_roll_rad_ = measured_roll;
            filtered_pitch_rad_ = measured_pitch;
            tilt_initialized_ = true;
        } else {
            filtered_roll_rad_ =
                (1.0 - tilt_alpha_) * filtered_roll_rad_ + tilt_alpha_ * measured_roll;
            filtered_pitch_rad_ =
                (1.0 - tilt_alpha_) * filtered_pitch_rad_ + tilt_alpha_ * measured_pitch;
        }

        roll_rad = filtered_roll_rad_;
        pitch_rad = filtered_pitch_rad_;
        return true;
    }

    double extractYawRad(const sensor_msgs::msg::Imu & msg) const
    {
        const auto & q = msg.orientation;
        if (!std::isfinite(q.x) || !std::isfinite(q.y) ||
            !std::isfinite(q.z) || !std::isfinite(q.w))
        {
            return 0.0;
        }

        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        if (quat.length2() < 1e-12) {
            return 0.0;
        }

        quat.normalize();

        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        return yaw;
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        sensor_msgs::msg::Imu output = *msg;

        const auto & q = msg->orientation;
        if (std::isfinite(q.x) && std::isfinite(q.y) &&
            std::isfinite(q.z) && std::isfinite(q.w))
        {
            tf2::Quaternion quat(q.x, q.y, q.z, q.w);
            if (quat.length2() >= 1e-12) {
                quat.normalize();
                output.orientation.x = quat.x();
                output.orientation.y = quat.y();
                output.orientation.z = quat.z();
                output.orientation.w = quat.w();
            }
        }

        if (override_tilt_from_accel_) {
            double roll_rad = 0.0;
            double pitch_rad = 0.0;
            updateTiltEstimate(*msg, roll_rad, pitch_rad);
            const double yaw_rad = extractYawRad(*msg);

            tf2::Quaternion quat;
            quat.setRPY(roll_rad, pitch_rad, yaw_rad);
            quat.normalize();

            output.orientation.x = quat.x();
            output.orientation.y = quat.y();
            output.orientation.z = quat.z();
            output.orientation.w = quat.w();
            output.orientation_covariance[0] = 1e-2;
            output.orientation_covariance[4] = 1e-2;
            output.orientation_covariance[8] = 1e-2;
        }

        imu_pub_->publish(output);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    std::string input_topic_;
    std::string output_topic_;
    double tilt_alpha_ = 0.2;
    bool override_tilt_from_accel_ = false;
    double filtered_roll_rad_ = 0.0;
    double filtered_pitch_rad_ = 0.0;
    bool tilt_initialized_ = false;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimImuNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
