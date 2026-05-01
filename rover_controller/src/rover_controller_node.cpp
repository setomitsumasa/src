#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>

namespace rover_controller
{

class Commander : public rclcpp::Node
{
public:
  explicit Commander(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("commander", options)
  {
    timer_period_ = 0.01;  // 50Hz

    sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&Commander::cmdVelCallback, this, std::placeholders::_1));
    pub_ = create_publisher<std_msgs::msg::Int16MultiArray>("uart_command", 10);

    rover_control_[0] = 0.0; // Rover steering Angle
    rover_control_[1] = 0.0; // Rover tire speed

    opposite_degree_R_id_ = 0x310;
    opposite_degree_L_id_ = 0x311;
    opposite_spped_id_ = 0x312;

    angle_can_id_ = 0x300;
    speed_can_id_ = 0x300;

    timer_ = create_wall_timer(
      std::chrono::duration<double>(timer_period_),
      std::bind(&Commander::timerCallback, this));
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    latest_twist_ = *msg;
  }

  void calculateAngleAndVelocity(
    const geometry_msgs::msg::Twist & twist,
    double & direction_angle, double & velocity)
  {
    double linear_x = twist.linear.x;
    double angular_z = twist.angular.z;

    velocity = linear_x;
    direction_angle = -angular_z;  // 右回りを正とするために符号を反転
  }

  void timerCallback()
  {
    double direction_angle, velocity;
    calculateAngleAndVelocity(latest_twist_, direction_angle, velocity);

    std_msgs::msg::Int16MultiArray rover_cmd;

    if (std::abs(direction_angle) < 0.3) {  // /cmd_velから受け取った-1~1の範囲の角度の値が -0.3 < 値 < 0.3なら
      rover_control_[0] = 180; // 180が角度0, ステアの角度 + 180 = 送るデータ
      rover_control_[1] = velocity * 100 * 3.4 + 120;  // (-1 ~ 1) * 10 * 3 + 120
    } else if (direction_angle >= 0.3) {  // /cmd_velから受け取った-1~1の範囲の角度の値が0.3より大きいなら右？へ下の式のスピードで曲がる
      rover_control_[0] = (-direction_angle * 180.0 / M_PI) * 0.4 + 180;
      rover_control_[1] = (velocity * 100 * 1.1 + std::abs(direction_angle) * 38) + 120; // 120がスピード0, 出したいスピード + 120 = 送るデータ
      angle_can_id_ = opposite_degree_R_id_;
    } else {  // /cmd_velから受け取った-1~1の範囲の角度の値が上にはまらない値なら左？へ下の式のスピードで曲がる
      rover_control_[0] = (-direction_angle * 180.0 / M_PI) * 0.4 + 180;
      rover_control_[1] = (velocity * 100 * 1.1 + std::abs(direction_angle) * 38) + 120;
      angle_can_id_ = opposite_degree_L_id_;
    }
    speed_can_id_ = opposite_spped_id_;

    if (rover_control_[1] > 120 + 40) {
      rover_control_[1] = 120 + 40;
    }

    // IDと制御値を順番にデータ配列に追加
    rover_cmd.data = {
      static_cast<int16_t>(angle_can_id_),
      static_cast<int16_t>(rover_control_[0]),
      static_cast<int16_t>(speed_can_id_),
      static_cast<int16_t>(rover_control_[1])
    };

    pub_->publish(rover_cmd); // -> uart_publisher -> TYPEC -> rover基板 -> CAN -> 各タイヤのマイコンで処理 -> ステアの角度とタイヤの速度に変換

    rover_control_[0] = 0;
    rover_control_[1] = 0;
  }

  double timer_period_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pub_;
  double rover_control_[2];
  int opposite_degree_R_id_;
  int opposite_degree_L_id_;
  int opposite_spped_id_;
  int angle_can_id_;
  int speed_can_id_;
  geometry_msgs::msg::Twist latest_twist_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rover_controller

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<rover_controller::Commander>(options);
  try {
    rclcpp::spin(node);
  } catch (const std::exception &) {
    // KeyboardInterrupt equivalent: just pass
  }
  rclcpp::shutdown();
  return 0;
}
