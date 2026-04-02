#include <memory>

#include "ares_nav2/yolo_tf_nav2_goal.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ares_nav2::YoloTfNav2Goal>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
