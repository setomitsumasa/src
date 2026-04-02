//
// ArUco marker TF を Nav2 のゴールとして publish するノード
//

#ifndef ARES_NAV2_ARUCO_NAV2_GOAL_HPP
#define ARES_NAV2_ARUCO_NAV2_GOAL_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <string>
#include <mutex>
#include <chrono>

namespace ares_nav2
{

class ArucoNav2Goal : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit ArucoNav2Goal(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onTimer();
  void sendGoal(const geometry_msgs::msg::PoseStamped & pose);
  void onGoalResponse(GoalHandleNavigateToPose::SharedPtr handle);
  void onResult(const GoalHandleNavigateToPose::WrappedResult & result);

  // ROS
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  void cancelCurrentGoal();

  // Parameters
  std::string target_frame_;
  std::string aruco_frame_;
  std::string robot_base_frame_;
  double goal_send_interval_sec_;
  double goal_update_threshold_;
  double goal_tolerance_;
  bool send_only_once_;
  std::string navigate_to_pose_action_;

  // State
  std::mutex goal_mutex_;
  GoalHandleNavigateToPose::SharedPtr current_goal_handle_;
  bool goal_sent_once_{false};
  rclcpp::Time last_goal_time_{0, 0, RCL_ROS_TIME};
  double last_goal_x_{0.0};
  double last_goal_y_{0.0};
  bool has_last_goal_{false};
};

}  // namespace ares_nav2

#endif  // ARES_NAV2_ARUCO_NAV2_GOAL_HPP
