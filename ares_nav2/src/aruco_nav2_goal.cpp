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
  declare_parameter<std::string>("robot_base_frame", "base_link");
  declare_parameter<double>("goal_send_interval_sec", 5.0);
  declare_parameter<double>("goal_update_threshold", 0.3);
  declare_parameter<double>("max_tf_age_sec", 1.0);
  declare_parameter<double>("goal_hold_time_sec", 5.0);
  declare_parameter<double>("goal_tolerance", 1.5);
  declare_parameter<bool>("send_only_once", false);
  declare_parameter<bool>("follow_any_detected_marker", false);
  declare_parameter<std::string>("navigate_to_pose_action", "navigate_to_pose");
  declare_parameter<double>("timer_period_sec", 0.2);
  declare_parameter<std::string>("target_marker_topic", "/aruco/target_marker_id");
  declare_parameter<std::string>("detected_marker_topic", "/aruco/id");
  declare_parameter<std::string>("goal_reached_topic", "/aruco/goal_reached");

  target_frame_ = get_parameter("target_frame").get_value<std::string>();
  aruco_frame_ = get_parameter("aruco_frame").get_value<std::string>();
  robot_base_frame_ = get_parameter("robot_base_frame").get_value<std::string>();
  goal_send_interval_sec_ = get_parameter("goal_send_interval_sec").get_value<double>();
  goal_update_threshold_ = get_parameter("goal_update_threshold").get_value<double>();
  max_tf_age_sec_ = get_parameter("max_tf_age_sec").get_value<double>();
  goal_hold_time_sec_ = get_parameter("goal_hold_time_sec").get_value<double>();
  goal_tolerance_ = get_parameter("goal_tolerance").get_value<double>();
  send_only_once_ = get_parameter("send_only_once").get_value<bool>();
  follow_any_detected_marker_ = get_parameter("follow_any_detected_marker").get_value<bool>();
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
    "aruco_nav2_goal: target_frame=%s, aruco_frame=%s, robot_base_frame=%s, interval=%.1fs, update_threshold=%.2fm, max_tf_age=%.2fs, hold=%.1fs, goal_tolerance=%.2fm, send_only_once=%d, follow_any_detected_marker=%d",
    target_frame_.c_str(), aruco_frame_.c_str(), robot_base_frame_.c_str(),
    goal_send_interval_sec_, goal_update_threshold_, max_tf_age_sec_, goal_hold_time_sec_,
    goal_tolerance_, send_only_once_, follow_any_detected_marker_);
}

void ArucoNav2Goal::resetTrackingState()
{
  cancelCurrentGoal();
  goal_sent_once_ = false;
  last_goal_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_goal_x_ = 0.0;
  last_goal_y_ = 0.0;
  has_last_goal_ = false;
  has_cached_target_pose_ = false;
  cached_target_pose_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
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
  const int marker_id = static_cast<int>(std::lround(msg->data));
  if (follow_any_detected_marker_ && (!has_detected_marker_id_ || detected_marker_id_ != marker_id)) {
    resetTrackingState();
  }
  detected_marker_id_ = marker_id;
  has_detected_marker_id_ = true;
}

void ArucoNav2Goal::onTimer()
{
  const bool marker_matches = has_detected_marker_id_ &&
    (follow_any_detected_marker_ || detected_marker_id_ == target_marker_id_);

  if (follow_any_detected_marker_) {
    if (!has_detected_marker_id_ && !has_cached_target_pose_) {
      return;
    }
  } else {
    if (target_marker_id_ < 0) {
      return;
    }
    if (!marker_matches && !has_cached_target_pose_) {
      return;
    }
  }

  if (waiting_after_goal_reached_) {
    return;
  }

  geometry_msgs::msg::PoseStamped target_pose;
  bool using_cached_pose = true;

  if (marker_matches) {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(
        target_frame_, aruco_frame_, tf2::TimePointZero);

      bool fresh_tf = true;
      const auto transform_stamp = rclcpp::Time(transform.header.stamp);
      if (transform_stamp.nanoseconds() > 0) {
        const double tf_age_sec = (now() - transform_stamp).seconds();
        if (tf_age_sec > max_tf_age_sec_) {
          fresh_tf = false;
          RCLCPP_DEBUG(
            get_logger(),
            "Keeping cached ArUco goal because current TF is stale (age=%.3fs > %.3fs)",
            tf_age_sec, max_tf_age_sec_);
        }
      } else if (follow_any_detected_marker_) {
        fresh_tf = false;
        RCLCPP_DEBUG(get_logger(), "Keeping cached ArUco goal because current TF has no timestamp");
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
      RCLCPP_DEBUG(get_logger(), "TF lookup failed: %s", ex.what());
    }
  }

  if (using_cached_pose) {
    if (!has_cached_target_pose_) {
      return;
    }
    target_pose = cached_target_pose_;
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Using cached ArUco goal: (%.2f, %.2f) in frame %s",
      target_pose.pose.position.x, target_pose.pose.position.y,
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
  const bool moved = !has_last_goal_ ||
    (std::hypot(x - last_goal_x_, y - last_goal_y_) > goal_update_threshold_);
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

bool ArucoNav2Goal::hasActiveGoal()
{
  std::lock_guard<std::mutex> lock(goal_mutex_);
  if (!current_goal_handle_) {
    return false;
  }

  const auto status = current_goal_handle_->get_status();
  return status == rclcpp_action::GoalStatus::STATUS_EXECUTING ||
    status == rclcpp_action::GoalStatus::STATUS_ACCEPTED;
}

bool ArucoNav2Goal::isTargetActive() const
{
  if (waiting_after_goal_reached_) {
    return false;
  }
  if (follow_any_detected_marker_) {
    return has_detected_marker_id_ || has_cached_target_pose_;
  }
  return target_marker_id_ >= 0;
}

bool ArucoNav2Goal::isRobotWithinGoalTolerance(
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
        "\033[32mReached ArUco goal by distance threshold: target is %.2fm away (tolerance %.2fm)\033[0m",
        distance_to_target, goal_tolerance_);
      return true;
    }
  } catch (const tf2::TransformException & ex) {
    RCLCPP_DEBUG(get_logger(), "Robot pose TF lookup failed for ArUco goal: %s", ex.what());
  }
  return false;
}

void ArucoNav2Goal::cancelCurrentGoal(bool suppress_cancel_result)
{
  std::lock_guard<std::mutex> lock(goal_mutex_);
  if (current_goal_handle_) {
    const auto status = current_goal_handle_->get_status();
    if (status == rclcpp_action::GoalStatus::STATUS_EXECUTING ||
        status == rclcpp_action::GoalStatus::STATUS_ACCEPTED) {
      if (suppress_cancel_result) {
        ++suppressed_cancel_results_;
      }
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
    "\033[32mSent Nav2 goal from ArUco TF: (%.2f, %.2f) in frame %s\033[0m",
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

void ArucoNav2Goal::markGoalReached()
{
  if (goal_hold_time_sec_ > 0.0) {
    RCLCPP_INFO(get_logger(), "Holding position for %.1f seconds at ArUco goal", goal_hold_time_sec_);
  }

  goal_hold_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(std::max(0.0, goal_hold_time_sec_))),
    [this]() {
      if (goal_hold_timer_) {
        goal_hold_timer_->cancel();
        goal_hold_timer_.reset();
      }

      publishGoalReached(true);

      waiting_after_goal_reached_ = false;
      if (follow_any_detected_marker_) {
        goal_sent_once_ = send_only_once_;
      } else {
        target_marker_id_ = -1;
        resetTrackingState();
      }
    });
}

void ArucoNav2Goal::publishGoalReached(bool reached)
{
  std_msgs::msg::Bool msg;
  msg.data = reached;
  goal_reached_pub_->publish(msg);
}

void ArucoNav2Goal::onResult(const GoalHandleNavigateToPose::WrappedResult & result)
{
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    current_goal_handle_.reset();
  }
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(get_logger(), "\033[32mReached ArUco goal\033[0m");
      waiting_after_goal_reached_ = true;
      markGoalReached();
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_WARN(get_logger(), "Goal aborted");
      if (has_cached_target_pose_ && isTargetActive()) {
        RCLCPP_WARN(
          get_logger(),
          "Keeping ArUco target active; will retry cached goal at (%.2f, %.2f)",
          cached_target_pose_.pose.position.x, cached_target_pose_.pose.position.y);
        break;
      }
      publishGoalReached(false);
      break;
    case rclcpp_action::ResultCode::CANCELED:
      if (suppressed_cancel_results_ > 0) {
        --suppressed_cancel_results_;
        RCLCPP_INFO(get_logger(), "Goal canceled by ArUco goal updater");
        break;
      }
      if (has_cached_target_pose_ && isTargetActive()) {
        RCLCPP_WARN(
          get_logger(),
          "ArUco goal was canceled while target is active; will continue toward cached goal at (%.2f, %.2f)",
          cached_target_pose_.pose.position.x, cached_target_pose_.pose.position.y);
        break;
      }
      RCLCPP_INFO(get_logger(), "Goal canceled");
      publishGoalReached(false);
      break;
    default:
      break;
  }
}

}  // namespace ares_nav2

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ares_nav2::ArucoNav2Goal)
