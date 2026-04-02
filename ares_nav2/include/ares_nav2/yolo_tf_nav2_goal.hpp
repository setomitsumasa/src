//
// Selected YOLO target TF を Nav2 goal として送るノード
//

#ifndef ARES_NAV2_YOLO_TF_NAV2_GOAL_HPP
#define ARES_NAV2_YOLO_TF_NAV2_GOAL_HPP

#include <chrono>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace ares_nav2
{

class YoloTfNav2Goal : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit YoloTfNav2Goal(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onTimer();
  void onTargetFrame(const std_msgs::msg::String::SharedPtr msg);
  void sendGoal(const geometry_msgs::msg::PoseStamped & pose);
  void onGoalResponse(GoalHandleNavigateToPose::SharedPtr handle);
  void onResult(const GoalHandleNavigateToPose::WrappedResult & result);
  void cancelCurrentGoal();
  void resetTrackingState();

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr target_frame_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;

  std::string target_frame_;
  std::string tracked_frame_;
  std::string inactive_value_;
  std::string target_frame_topic_;
  std::string goal_reached_topic_;
  std::string navigate_to_pose_action_;
  double goal_send_interval_sec_;
  double goal_update_threshold_;
  double max_tf_age_sec_;
  bool send_only_once_;

  std::mutex goal_mutex_;
  GoalHandleNavigateToPose::SharedPtr current_goal_handle_;
  bool goal_sent_once_{false};
  rclcpp::Time last_goal_time_{0, 0, RCL_ROS_TIME};
  double last_goal_x_{0.0};
  double last_goal_y_{0.0};
  bool has_last_goal_{false};
};

}  // namespace ares_nav2

#endif  // ARES_NAV2_YOLO_TF_NAV2_GOAL_HPP
