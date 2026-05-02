//
// Selected YOLO target TF を map/odom 上の Nav2 goal に変換する
//

#include "ares_nav2/yolo_tf_nav2_goal.hpp"

#include <cmath>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/create_timer_ros.h>

namespace ares_nav2
{

YoloTfNav2Goal::YoloTfNav2Goal(const rclcpp::NodeOptions & options)
: rclcpp::Node("yolo_tf_nav2_goal", options)
{
  declare_parameter<std::string>("target_frame", "map");
  declare_parameter<std::string>("robot_base_frame", "base_link");
  declare_parameter<std::string>("target_frame_topic", "/yolo/target_frame");
  declare_parameter<std::string>("goal_reached_topic", "/yolo/goal_reached");
  declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  declare_parameter<std::string>("inactive_value", "disable");
  declare_parameter<std::string>("navigate_to_pose_action", "navigate_to_pose");
  declare_parameter<double>("goal_send_interval_sec", 1.0);
  declare_parameter<double>("goal_update_threshold", 0.3);
  declare_parameter<double>("max_tf_age_sec", 0.5);
  declare_parameter<double>("goal_hold_time_sec", 5.0);
  declare_parameter<double>("goal_tolerance", 1.5);
  declare_parameter<double>("timer_period_sec", 0.2);
  declare_parameter<bool>("send_only_once", false);

  target_frame_ = get_parameter("target_frame").get_value<std::string>();
  robot_base_frame_ = get_parameter("robot_base_frame").get_value<std::string>();
  target_frame_topic_ = get_parameter("target_frame_topic").get_value<std::string>();
  goal_reached_topic_ = get_parameter("goal_reached_topic").get_value<std::string>();
  cmd_vel_topic_ = get_parameter("cmd_vel_topic").get_value<std::string>();
  inactive_value_ = get_parameter("inactive_value").get_value<std::string>();
  navigate_to_pose_action_ = get_parameter("navigate_to_pose_action").get_value<std::string>();
  goal_send_interval_sec_ = get_parameter("goal_send_interval_sec").get_value<double>();
  goal_update_threshold_ = get_parameter("goal_update_threshold").get_value<double>();
  max_tf_age_sec_ = get_parameter("max_tf_age_sec").get_value<double>();
  goal_hold_time_sec_ = get_parameter("goal_hold_time_sec").get_value<double>();
  goal_tolerance_ = get_parameter("goal_tolerance").get_value<double>();
  const double timer_period = get_parameter("timer_period_sec").get_value<double>();
  send_only_once_ = get_parameter("send_only_once").get_value<bool>();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(), get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  action_client_ = rclcpp_action::create_client<NavigateToPose>(this, navigate_to_pose_action_);
  target_frame_sub_ = create_subscription<std_msgs::msg::String>(
    target_frame_topic_, 10,
    std::bind(&YoloTfNav2Goal::onTargetFrame, this, std::placeholders::_1));
  goal_reached_pub_ = create_publisher<std_msgs::msg::Bool>(goal_reached_topic_, 10);
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

  timer_ = create_wall_timer(
    std::chrono::duration<double>(timer_period),
    std::bind(&YoloTfNav2Goal::onTimer, this));

  RCLCPP_INFO(
    get_logger(),
    "yolo_tf_nav2_goal: target_frame=%s, robot_base_frame=%s, target_frame_topic=%s, inactive_value=%s, hold=%.1fs, goal_tolerance=%.2fm",
    target_frame_.c_str(), robot_base_frame_.c_str(), target_frame_topic_.c_str(),
    inactive_value_.c_str(), goal_hold_time_sec_, goal_tolerance_);
}

void YoloTfNav2Goal::resetTrackingState()
{
  cancelCurrentGoal();
  if (goal_hold_timer_) {
    goal_hold_timer_->cancel();
    goal_hold_timer_.reset();
  }
  if (stop_cmd_timer_) {
    stop_cmd_timer_->cancel();
    stop_cmd_timer_.reset();
  }
  waiting_after_goal_reached_ = false;
  goal_sent_once_ = false;
  last_goal_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_goal_x_ = 0.0;
  last_goal_y_ = 0.0;
  has_last_goal_ = false;
  has_cached_target_pose_ = false;
  cached_target_pose_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

void YoloTfNav2Goal::onTargetFrame(const std_msgs::msg::String::SharedPtr msg)
{
  if (tracked_frame_ == msg->data) {
    return;
  }

  tracked_frame_ = msg->data;
  resetTrackingState();

  if (tracked_frame_.empty() || tracked_frame_ == inactive_value_) {
    RCLCPP_INFO(get_logger(), "YOLO target disabled");
    tracked_frame_.clear();
  } else {
    RCLCPP_INFO(get_logger(), "Tracking YOLO TF frame '%s'", tracked_frame_.c_str());
  }
}

void YoloTfNav2Goal::onTimer()
{
  if (tracked_frame_.empty() || waiting_after_goal_reached_) {
    return;
  }

  geometry_msgs::msg::PoseStamped target_pose;
  bool using_cached_pose = true;

  try {
    const auto transform = tf_buffer_->lookupTransform(target_frame_, tracked_frame_, tf2::TimePointZero);
    bool fresh_tf = true;
    if (max_tf_age_sec_ > 0.0 && transform.header.stamp.sec != 0) {
      const double tf_age = (now() - rclcpp::Time(transform.header.stamp)).seconds();
      if (tf_age > max_tf_age_sec_) {
        fresh_tf = false;
        RCLCPP_DEBUG(
          get_logger(),
          "Keeping cached YOLO goal for '%s' because current TF is stale (age=%.3fs)",
          tracked_frame_.c_str(), tf_age);
      }
    }

    if (fresh_tf) {
      target_pose.header = transform.header;
      target_pose.header.stamp = now();
      target_pose.pose.position.x = transform.transform.translation.x;
      target_pose.pose.position.y = transform.transform.translation.y;
      target_pose.pose.position.z = transform.transform.translation.z;
      target_pose.pose.orientation = transform.transform.rotation;
      cached_target_pose_ = target_pose;
      cached_target_pose_time_ = now();
      has_cached_target_pose_ = true;
      using_cached_pose = false;
    }
  } catch (const tf2::TransformException & ex) {
    RCLCPP_DEBUG(get_logger(), "TF lookup failed for '%s': %s", tracked_frame_.c_str(), ex.what());
  }

  if (using_cached_pose) {
    if (!has_cached_target_pose_) {
      return;
    }
    target_pose = cached_target_pose_;
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Using cached YOLO goal for '%s': (%.2f, %.2f) in frame %s",
      tracked_frame_.c_str(), target_pose.pose.position.x, target_pose.pose.position.y,
      target_pose.header.frame_id.c_str());
  }

  if (isRobotWithinGoalTolerance(target_pose)) {
    waiting_after_goal_reached_ = true;
    cancelCurrentGoal();
    markGoalReached();
    return;
  }

  const double x = target_pose.pose.position.x;
  const double y = target_pose.pose.position.y;
  const rclcpp::Time current_time = now();
  const bool interval_ok = (current_time - last_goal_time_).seconds() >= goal_send_interval_sec_;
  const bool moved =
    !has_last_goal_ || (std::hypot(x - last_goal_x_, y - last_goal_y_) > goal_update_threshold_);
  const bool active_goal = hasActiveGoal();

  if (send_only_once_ && goal_sent_once_ && active_goal) {
    return;
  }

  if (active_goal && (!interval_ok || !moved)) {
    return;
  }
  if (!active_goal && has_last_goal_ && !interval_ok && !moved) {
    return;
  }

  if (active_goal) {
    cancelCurrentGoal();
  }

  target_pose.header.stamp = now();
  sendGoal(target_pose);
  last_goal_time_ = current_time;
  last_goal_x_ = x;
  last_goal_y_ = y;
  has_last_goal_ = true;
  if (send_only_once_) {
    goal_sent_once_ = true;
  }
}

bool YoloTfNav2Goal::hasActiveGoal()
{
  std::lock_guard<std::mutex> lock(goal_mutex_);
  if (!current_goal_handle_) {
    return false;
  }

  const auto status = current_goal_handle_->get_status();
  return status == rclcpp_action::GoalStatus::STATUS_EXECUTING ||
    status == rclcpp_action::GoalStatus::STATUS_ACCEPTED;
}

bool YoloTfNav2Goal::isTargetActive() const
{
  return !tracked_frame_.empty() && !waiting_after_goal_reached_;
}

bool YoloTfNav2Goal::isRobotWithinGoalTolerance(
  const geometry_msgs::msg::PoseStamped & pose) const
{
  try {
    const auto robot_transform = tf_buffer_->lookupTransform(
      pose.header.frame_id, robot_base_frame_, tf2::TimePointZero);
    const double distance_to_target = std::hypot(
      pose.pose.position.x - robot_transform.transform.translation.x,
      pose.pose.position.y - robot_transform.transform.translation.y);

    if (distance_to_target <= goal_tolerance_) {
      RCLCPP_INFO(
        get_logger(),
        "\033[32mReached YOLO goal by distance threshold: target is %.2fm away (tolerance %.2fm)\033[0m",
        distance_to_target, goal_tolerance_);
      return true;
    }
  } catch (const tf2::TransformException & ex) {
    RCLCPP_DEBUG(
      get_logger(), "Robot pose TF lookup failed for YOLO goal '%s': %s",
      tracked_frame_.c_str(), ex.what());
  }
  return false;
}

void YoloTfNav2Goal::cancelCurrentGoal(bool suppress_cancel_result)
{
  std::lock_guard<std::mutex> lock(goal_mutex_);
  if (current_goal_handle_) {
    const auto status = current_goal_handle_->get_status();
    if (status == rclcpp_action::GoalStatus::STATUS_EXECUTING ||
        status == rclcpp_action::GoalStatus::STATUS_ACCEPTED)
    {
      if (suppress_cancel_result) {
        ++suppressed_cancel_results_;
      }
      action_client_->async_cancel_goal(current_goal_handle_);
      RCLCPP_DEBUG(get_logger(), "Canceled current YOLO TF goal to update target position");
    }
    current_goal_handle_.reset();
  }
}

void YoloTfNav2Goal::sendGoal(const geometry_msgs::msg::PoseStamped & pose)
{
  if (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "NavigateToPose action server not available");
    return;
  }

  NavigateToPose::Goal goal;
  goal.pose = pose;

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;
  opts.goal_response_callback =
    std::bind(&YoloTfNav2Goal::onGoalResponse, this, std::placeholders::_1);
  opts.result_callback = std::bind(&YoloTfNav2Goal::onResult, this, std::placeholders::_1);

  action_client_->async_send_goal(goal, opts);
  RCLCPP_INFO(
    get_logger(), "Sent Nav2 goal from YOLO TF '%s': (%.2f, %.2f) in frame %s",
    tracked_frame_.c_str(), pose.pose.position.x, pose.pose.position.y, pose.header.frame_id.c_str());
}

void YoloTfNav2Goal::onGoalResponse(GoalHandleNavigateToPose::SharedPtr handle)
{
  if (!handle) {
    RCLCPP_ERROR(get_logger(), "YOLO TF goal was rejected by Nav2");
    return;
  }

  std::lock_guard<std::mutex> lock(goal_mutex_);
  current_goal_handle_ = handle;
  RCLCPP_INFO(get_logger(), "YOLO TF goal accepted");
}

void YoloTfNav2Goal::markGoalReached()
{
  publishZeroCmdVel();
  if (stop_cmd_timer_) {
    stop_cmd_timer_->cancel();
    stop_cmd_timer_.reset();
  }
  stop_cmd_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&YoloTfNav2Goal::publishZeroCmdVel, this));

  if (goal_hold_time_sec_ > 0.0) {
    RCLCPP_INFO(get_logger(), "Holding position for %.1f seconds at YOLO goal", goal_hold_time_sec_);
  }

  goal_hold_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(std::max(0.0, goal_hold_time_sec_))),
    [this]() {
      if (goal_hold_timer_) {
        goal_hold_timer_->cancel();
        goal_hold_timer_.reset();
      }
      publishZeroCmdVel();
      if (stop_cmd_timer_) {
        stop_cmd_timer_->cancel();
        stop_cmd_timer_.reset();
      }

      publishGoalReached(true);

      waiting_after_goal_reached_ = false;
      tracked_frame_.clear();
      resetTrackingState();
    });
}

void YoloTfNav2Goal::publishZeroCmdVel()
{
  if (!cmd_vel_pub_) {
    return;
  }
  geometry_msgs::msg::Twist stop_cmd;
  cmd_vel_pub_->publish(stop_cmd);
}

void YoloTfNav2Goal::publishGoalReached(bool reached)
{
  std_msgs::msg::Bool msg;
  msg.data = reached;
  goal_reached_pub_->publish(msg);
}

void YoloTfNav2Goal::onResult(const GoalHandleNavigateToPose::WrappedResult & result)
{
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    current_goal_handle_.reset();
  }

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(get_logger(), "\033[32mReached YOLO TF goal\033[0m");
      waiting_after_goal_reached_ = true;
      markGoalReached();
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_WARN(get_logger(), "YOLO TF goal aborted");
      if (has_cached_target_pose_ && isTargetActive()) {
        RCLCPP_WARN(
          get_logger(),
          "Keeping YOLO target '%s' active; will retry cached goal at (%.2f, %.2f)",
          tracked_frame_.c_str(), cached_target_pose_.pose.position.x,
          cached_target_pose_.pose.position.y);
        break;
      }
      publishGoalReached(false);
      break;
    case rclcpp_action::ResultCode::CANCELED:
      if (suppressed_cancel_results_ > 0) {
        --suppressed_cancel_results_;
        RCLCPP_INFO(get_logger(), "YOLO TF goal canceled by goal updater");
        break;
      }
      if (has_cached_target_pose_ && isTargetActive()) {
        RCLCPP_WARN(
          get_logger(),
          "YOLO TF goal was canceled while target '%s' is active; will continue toward cached goal at (%.2f, %.2f)",
          tracked_frame_.c_str(), cached_target_pose_.pose.position.x,
          cached_target_pose_.pose.position.y);
        break;
      }
      RCLCPP_INFO(get_logger(), "YOLO TF goal canceled");
      publishGoalReached(false);
      break;
    default:
      break;
  }
}

}  // namespace ares_nav2
