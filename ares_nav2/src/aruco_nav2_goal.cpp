//
// ArUco marker の TF を map/odom 座標で取得し、Nav2 の NavigateToPose ゴールとして送信する
//

#include "ares_nav2/aruco_nav2_goal.hpp"
#include <tf2_ros/create_timer_ros.h>
#include <cmath>
#include <std_msgs/msg/bool.hpp>

namespace ares_nav2
{

ArucoNav2Goal::ArucoNav2Goal(const rclcpp::NodeOptions & options)
: rclcpp::Node("aruco_nav2_goal", options)
{
  declare_parameter<std::string>("target_frame", "map");
  declare_parameter<std::string>("aruco_frame", "aruco_marker");
  declare_parameter<double>("goal_send_interval_sec", 5.0);
  declare_parameter<double>("goal_update_threshold", 0.3);
  declare_parameter<bool>("send_only_once", false);
  declare_parameter<std::string>("navigate_to_pose_action", "navigate_to_pose");
  declare_parameter<double>("timer_period_sec", 0.2);
  declare_parameter<std::string>("target_marker_topic", "/aruco/target_marker_id");
  declare_parameter<std::string>("detected_marker_topic", "/aruco/id");
  declare_parameter<std::string>("goal_reached_topic", "/aruco/goal_reached");

  target_frame_ = get_parameter("target_frame").get_value<std::string>();
  aruco_frame_ = get_parameter("aruco_frame").get_value<std::string>();
  goal_send_interval_sec_ = get_parameter("goal_send_interval_sec").get_value<double>();
  goal_update_threshold_ = get_parameter("goal_update_threshold").get_value<double>();
  send_only_once_ = get_parameter("send_only_once").get_value<bool>();
  navigate_to_pose_action_ = get_parameter("navigate_to_pose_action").get_value<std::string>();
  target_marker_topic_ = get_parameter("target_marker_topic").get_value<std::string>();
  detected_marker_topic_ = get_parameter("detected_marker_topic").get_value<std::string>();
  goal_reached_topic_ = get_parameter("goal_reached_topic").get_value<std::string>();
  const double timer_period = get_parameter("timer_period_sec").get_value<double>();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(), get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  action_client_ = rclcpp_action::create_client<NavigateToPose>(this, navigate_to_pose_action_);
  target_marker_sub_ = create_subscription<std_msgs::msg::Int32>(
    target_marker_topic_, 10,
    std::bind(&ArucoNav2Goal::onTargetMarkerId, this, std::placeholders::_1));
  detected_marker_sub_ = create_subscription<std_msgs::msg::Float32>(
    detected_marker_topic_, 10,
    std::bind(&ArucoNav2Goal::onDetectedMarkerId, this, std::placeholders::_1));
  goal_reached_pub_ = create_publisher<std_msgs::msg::Bool>(goal_reached_topic_, 10);

  timer_ = create_wall_timer(
    std::chrono::duration<double>(timer_period),
    std::bind(&ArucoNav2Goal::onTimer, this));

  RCLCPP_INFO(get_logger(),
    "aruco_nav2_goal: target_frame=%s, aruco_frame=%s, interval=%.1fs, update_threshold=%.2fm, send_only_once=%d",
    target_frame_.c_str(), aruco_frame_.c_str(), goal_send_interval_sec_, goal_update_threshold_, send_only_once_);
}

void ArucoNav2Goal::resetTrackingState()
{
  cancelCurrentGoal();
  goal_sent_once_ = false;
  last_goal_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_goal_x_ = 0.0;
  last_goal_y_ = 0.0;
  has_last_goal_ = false;
}

void ArucoNav2Goal::onTargetMarkerId(const std_msgs::msg::Int32::SharedPtr msg)
{
  if (target_marker_id_ == msg->data) {
    return;
  }

  target_marker_id_ = msg->data;
  resetTrackingState();

  if (target_marker_id_ < 0) {
    RCLCPP_INFO(get_logger(), "ArUco target disabled");
  } else {
    RCLCPP_INFO(get_logger(), "Tracking ArUco marker_id=%d", target_marker_id_);
  }
}

void ArucoNav2Goal::onDetectedMarkerId(const std_msgs::msg::Float32::SharedPtr msg)
{
  detected_marker_id_ = static_cast<int>(std::lround(msg->data));
  has_detected_marker_id_ = true;
}

void ArucoNav2Goal::onTimer()
{
  if (target_marker_id_ < 0) {
    return;
  }

  if (!has_detected_marker_id_ || detected_marker_id_ != target_marker_id_) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
      target_frame_, aruco_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_DEBUG(get_logger(), "TF lookup failed: %s", ex.what());
    return;
  }

  if (send_only_once_ && goal_sent_once_) {
    return;
  }

  const double x = transform.transform.translation.x;
  const double y = transform.transform.translation.y;

  const rclcpp::Time current_time = now();
  const bool interval_ok = (current_time - last_goal_time_).seconds() >= goal_send_interval_sec_;
  const bool moved = !has_last_goal_ ||
    (std::hypot(x - last_goal_x_, y - last_goal_y_) > goal_update_threshold_);

  if (!interval_ok || !moved) {
    return;
  }

  cancelCurrentGoal();

  geometry_msgs::msg::PoseStamped pose;
  pose.header = transform.header;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = transform.transform.translation.z;
  pose.pose.orientation = transform.transform.rotation;

  sendGoal(pose);
  last_goal_time_ = current_time;
  last_goal_x_ = x;
  last_goal_y_ = y;
  has_last_goal_ = true;
  if (send_only_once_) {
    goal_sent_once_ = true;
  }
}

void ArucoNav2Goal::cancelCurrentGoal()
{
  std::lock_guard<std::mutex> lock(goal_mutex_);
  if (current_goal_handle_) {
    const auto status = current_goal_handle_->get_status();
    if (status == rclcpp_action::GoalStatus::STATUS_EXECUTING ||
        status == rclcpp_action::GoalStatus::STATUS_ACCEPTED) {
      action_client_->async_cancel_goal(current_goal_handle_);
      RCLCPP_DEBUG(get_logger(), "Canceled current goal to update to new TF position");
    }
    current_goal_handle_.reset();
  }
}

void ArucoNav2Goal::sendGoal(const geometry_msgs::msg::PoseStamped & pose)
{
  if (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "NavigateToPose action server not available");
    return;
  }

  NavigateToPose::Goal goal;
  goal.pose = pose;

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;
  opts.goal_response_callback = std::bind(&ArucoNav2Goal::onGoalResponse, this, std::placeholders::_1);
  opts.result_callback = std::bind(&ArucoNav2Goal::onResult, this, std::placeholders::_1);

  action_client_->async_send_goal(goal, opts);
  RCLCPP_INFO(get_logger(),
    "Sent Nav2 goal from ArUco TF: (%.2f, %.2f) in frame %s",
    pose.pose.position.x, pose.pose.position.y, pose.header.frame_id.c_str());
}

void ArucoNav2Goal::onGoalResponse(GoalHandleNavigateToPose::SharedPtr handle)
{
  if (!handle) {
    RCLCPP_ERROR(get_logger(), "Goal was rejected by Nav2");
    return;
  }
  std::lock_guard<std::mutex> lock(goal_mutex_);
  current_goal_handle_ = handle;
  RCLCPP_INFO(get_logger(), "Nav2 goal accepted");
}

void ArucoNav2Goal::onResult(const GoalHandleNavigateToPose::WrappedResult & result)
{
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    current_goal_handle_.reset();
  }
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(get_logger(), "Reached ArUco goal");
      {
        std_msgs::msg::Bool msg;
        msg.data = true;
        goal_reached_pub_->publish(msg);
      }
      target_marker_id_ = -1;
      resetTrackingState();
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_WARN(get_logger(), "Goal aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(get_logger(), "Goal canceled");
      break;
    default:
      break;
  }
}

}  // namespace ares_nav2

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ares_nav2::ArucoNav2Goal)
