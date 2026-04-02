//
// ArUco Nav2 Goal ノードのエントリポイント
//

#include "ares_nav2/aruco_nav2_goal.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<ares_nav2::ArucoNav2Goal>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
