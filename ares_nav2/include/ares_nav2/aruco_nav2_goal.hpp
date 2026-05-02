//
// ArUco marker TF を Nav2 のゴールとして publish するノード
//

#ifndef ARES_NAV2_ARUCO_NAV2_GOAL_HPP
#define ARES_NAV2_ARUCO_NAV2_GOAL_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
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
  void onTargetMarkerId(const std_msgs::msg::Int32::SharedPtr msg);
  void onDetectedMarkerId(const std_msgs::msg::Float32::SharedPtr msg);
  void resetTrackingState();
  void markGoalReached();
  void publishZeroCmdVel();
  void publishGoalReached(bool reached);
  bool hasActiveGoal();
  bool isTargetActive() const;
  bool isRobotWithinGoalTolerance(const geometry_msgs::msg::PoseStamped & pose) const;

  // ROS
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr goal_hold_timer_;
  rclcpp::TimerBase::SharedPtr stop_cmd_timer_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr target_marker_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr detected_marker_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  void cancelCurrentGoal(bool suppress_cancel_result = true);

  // Parameters
  std::string target_frame_;
  std::string aruco_frame_;
  std::string robot_base_frame_;
  double goal_send_interval_sec_;
  double goal_update_threshold_;
  double max_tf_age_sec_;
  double goal_hold_time_sec_;
  double goal_tolerance_;
  bool send_only_once_;
  bool follow_any_detected_marker_;
  std::string navigate_to_pose_action_;
  std::string target_marker_topic_;
  std::string detected_marker_topic_;
  std::string goal_reached_topic_;
  std::string cmd_vel_topic_;

  // State
  std::mutex goal_mutex_;
  GoalHandleNavigateToPose::SharedPtr current_goal_handle_;
  bool goal_sent_once_{false};
  rclcpp::Time last_goal_time_{0, 0, RCL_ROS_TIME};
  double last_goal_x_{0.0};
  double last_goal_y_{0.0};
  bool has_last_goal_{false};
  int target_marker_id_{-1};
  int detected_marker_id_{-1};
  bool has_detected_marker_id_{false};
  bool waiting_after_goal_reached_{false};
  int suppressed_cancel_results_{0};
  geometry_msgs::msg::PoseStamped cached_target_pose_;
  bool has_cached_target_pose_{false};
  rclcpp::Time cached_target_pose_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace ares_nav2

#endif  // ARES_NAV2_ARUCO_NAV2_GOAL_HPP
