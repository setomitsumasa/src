//
// Created by karisora on 2026/02/18.
// Rviz上では赤色軸が前方向
//

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <array>
#include <limits>
#include <cmath>

/**
 * UART から流れてくる `/uart_data` (Float64MultiArray) を購読し、
 * ID に応じて IMU の各成分を更新して `sensor_msgs::msg::Imu` を publish するノード。
 *
 * ID 割り当て:
 *   400 -> gyro.x
 *   401 -> gyro.y
 *   402 -> gyro.z
 *   403 -> acc.x
 *   404 -> acc.y
 *   405 -> acc.z
 *   412 -> yaw (0-360deg, 北=0deg)
 *   406 -> mag.x  (Imu メッセージには直接載せないため、ここでは未使用)
 *   407 -> mag.y  (同上)
 *   408 -> mag.z  (同上)
 *
 * Python 側 (`uart_control/serial_subscriber.py`) では
 *   Float64MultiArray.data = [ID, DATA]
 * を publish している前提。
 */
class UartImuNode : public rclcpp::Node
{
public:
    UartImuNode()
    : rclcpp::Node("uart_imu_node")
    {
        using std::placeholders::_1;

        // `/uart_data` を購読
        uart_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "uart_data",
                rclcpp::QoS(50),
                std::bind(&UartImuNode::uartCallback, this, _1));

        // IMU データを publish するトピック
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", rclcpp::QoS(50));

        // publish 周期（Hz）: sensor_tf.cpp と同様にパラメータ指定可
        const int publish_rate = this->declare_parameter<int>("publish_rate", 50);
        tilt_alpha_ = this->declare_parameter<double>("tilt_alpha", 0.2);
        timer_ = this->create_wall_timer(
                std::chrono::milliseconds(1000 / publish_rate),
                std::bind(&UartImuNode::publishImu, this));

        // NaN 初期化（未受信かどうかの判定用）
        for (double &v : gyro_)  v = std::numeric_limits<double>::quiet_NaN();
        for (double &v : accel_) v = std::numeric_limits<double>::quiet_NaN();
        yaw_deg_ = std::numeric_limits<double>::quiet_NaN();

        RCLCPP_INFO(
            this->get_logger(),
            "UartImuNode started. Subscribing /uart_data and publishing Imu on /imu/data");
    }

private:
    // コンパス方位(度, 0-360, 北=0, 時計回り)から
    // ROS のヨー角（ENU, Z軸まわり）を生成
    double headingDegToYawRad(double heading_deg) const
    {
        // heading: 北=0°, 東=90° の時計回り
        // ROS yaw: X=East, Y=North, yaw=0=East, 反時計回りが正
        // → yaw[rad] = (90 - heading_deg) * pi/180
        double yaw_rad = (90.0 - heading_deg) * M_PI / 180.0;
        // [-pi, pi] に正規化（必須ではないが念のため）
        return std::fmod(yaw_rad + 3.0 * M_PI, 2.0 * M_PI) - M_PI;
    }

    void rpyToQuaternion(double roll_rad, double pitch_rad, double yaw_rad,
                         double &qx, double &qy, double &qz, double &qw) const
    {
        tf2::Quaternion quat;
        quat.setRPY(roll_rad, pitch_rad, yaw_rad);
        quat.normalize();
        qx = quat.x();
        qy = quat.y();
        qz = quat.z();
        qw = quat.w();
    }

    bool updateTiltEstimate(double &roll_rad, double &pitch_rad)
    {
        const double ax = accel_[0];
        const double ay = accel_[1];
        const double az = accel_[2];

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

    void uartCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 2) {
            return;
        }

        const int id = static_cast<int>(msg->data[0]);
        const double value = msg->data[1];

        switch (id) {
        case 400: // gyro.x
            gyro_[0] = value;
            break;
        case 401: // gyro.y
            gyro_[1] = value;
            break;
        case 402: // gyro.z
            gyro_[2] = value;
            break;
        case 403: // acc.x
            accel_[0] = value;
            break;
        case 404: // acc.y
            accel_[1] = value;
            break;
        case 405: // acc.z
            accel_[2] = value;
            break;
        case 412: // yaw [deg] (0-360, 北=0deg)
            yaw_deg_ = value;
            break;
        default:
            // 他の ID は無視
            return;
        }
    }

    // 一定周期で IMU メッセージを publish（sensor_tf.cpp の timer スタイルに合わせる）
    void publishImu()
    {

        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = this->get_clock()->now();
        imu_msg.header.frame_id = "imu_link";  // 必要に応じて変更

        double roll_rad = 0.0;
        double pitch_rad = 0.0;
        updateTiltEstimate(roll_rad, pitch_rad);

        double yaw_rad = 0.0;
        if (std::isfinite(yaw_deg_)) {
            yaw_rad = headingDegToYawRad(yaw_deg_);
        }

        // `/uart_data` の加速度から roll/pitch、ID 412 のヨー角から yaw を生成
        rpyToQuaternion(roll_rad, pitch_rad, yaw_rad,
                        imu_msg.orientation.x,
                        imu_msg.orientation.y,
                        imu_msg.orientation.z,
                        imu_msg.orientation.w);
        imu_msg.orientation_covariance[0] = 1e-2;
        imu_msg.orientation_covariance[4] = 1e-2;
        imu_msg.orientation_covariance[8] = 1e-2;

        // 角速度
        imu_msg.angular_velocity.x = gyro_[0];
        imu_msg.angular_velocity.y = gyro_[1];
        imu_msg.angular_velocity.z = gyro_[2];

        // 加速度
        imu_msg.linear_acceleration.x = accel_[0];
        imu_msg.linear_acceleration.y = accel_[1];
        imu_msg.linear_acceleration.z = accel_[2];

        imu_pub_->publish(imu_msg);
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr uart_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // gyro[x, y, z], accel[x, y, z]
    std::array<double, 3> gyro_;
    std::array<double, 3> accel_;

    // ヘディング（度）
    double yaw_deg_;
    double tilt_alpha_ = 0.2;
    double filtered_roll_rad_ = 0.0;
    double filtered_pitch_rad_ = 0.0;
    bool tilt_initialized_ = false;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UartImuNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
