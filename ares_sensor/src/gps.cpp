//
// Created by karisora on 2025/09/12.
//

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

/**
 * UART から流れてくる `/uart_data` (Float64MultiArray) を購読し、
 * ID=415 を緯度(latitude)、ID=416 を経度(longitude)として解釈して
 * `sensor_msgs::msg::NavSatFix` 型で publish するノード。
 *
 * Python 側 (`uart_control/serial_subscriber.py`) では
 *   Float64MultiArray.data = [ID, DATA]
 * という 2 要素配列を publish している前提。
 */
class UartGpsNode : public rclcpp::Node
{
public:
    UartGpsNode()
    : rclcpp::Node("uart_gps_node"),
      latitude_(std::numeric_limits<double>::quiet_NaN()),
      longitude_(std::numeric_limits<double>::quiet_NaN())
    {
        using std::placeholders::_1;

        // `/uart_data` を購読 (ID, DATA)
        uart_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "uart_data",
                rclcpp::QoS(10),
                std::bind(&UartGpsNode::uartCallback, this, _1));

        // GPS として publish するトピック名（必要に応じて変更）
        gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
                "gps/fix",
                rclcpp::QoS(10));

        RCLCPP_INFO(this->get_logger(),
                    "UartGpsNode started. Subscribing /uart_data and publishing NavSatFix on /gps/fix");
    }

private:
    void uartCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 2) {
            RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    5000,
                    "Received uart_data with size %zu (< 2). Skipping.",
                    msg->data.size());
            return;
        }

        const auto id = static_cast<int>(msg->data[0]);
        const double value = msg->data[1];

        // ID=415 -> latitude、ID=416 -> longitude
        if (id == 415) {
            latitude_ = value;
        } else if (id == 416) {
            longitude_ = value;
        } else {
            // 他の ID はこのノードでは使用しない
            return;
        }

        // 緯度・経度が両方そろったタイミングで NavSatFix を publish
        if (std::isnan(latitude_) || std::isnan(longitude_)) {
            return;
        }

        sensor_msgs::msg::NavSatFix fix_msg;
        fix_msg.header.stamp = this->get_clock()->now();
        fix_msg.header.frame_id = "gps_link";  // navsat_transform が base_link -> gps_link の TF を参照する

        fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        fix_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

        fix_msg.latitude = latitude_;
        fix_msg.longitude = longitude_;
        fix_msg.altitude = 0.0;  // 高度が無い場合は 0 など適当な値にしておく

        // 共分散は未知とする
        fix_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

        gps_pub_->publish(fix_msg);
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr uart_sub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;

    double latitude_;
    double longitude_;
};

// ライブラリアイテムとしてこのファイルがリンクされるだけでも問題ないが、
// 単体ノードとして起動できるよう main も用意しておく。
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UartGpsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}