//
// Created by karisora on 2025/09/10.
// To Do:  add spiral search, 現在のcodeだとtrueでもspiral modeにならない
//

#include "ares_nav2/geo_utils.hpp"
#include "ares_nav2/waypoint_loader.hpp"
#include "ares_nav2/gps_waypoint_follower.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <thread>
#include <cmath>
#include <sstream>
#include <string>
#include <tf2/time.h>

using namespace std::chrono_literals;

namespace ares_nav2 {
    GPSWaypointFollower::GPSWaypointFollower(
        const std::string& waypoint_yaml_path,
        const SpiralParams& spiral_params)
            : rclcpp::Node("gps_waypoint_follower"),
              action_client_(rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose")),
                spiral_params_(spiral_params) {
        waypoints_ = load_waypoints(waypoint_yaml_path);
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "gps/fix", 10,
            std::bind(&GPSWaypointFollower::onGpsFix, this, std::placeholders::_1));
        marker_detected_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/aruco/id", 10,
            std::bind(&GPSWaypointFollower::onMarkerDetected, this, std::placeholders::_1));
        aruco_goal_reached_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/aruco/goal_reached", 10,
            std::bind(&GPSWaypointFollower::onArucoGoalReached, this, std::placeholders::_1));
        yolo_goal_reached_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/yolo/goal_reached", 10,
            std::bind(&GPSWaypointFollower::onYoloGoalReached, this, std::placeholders::_1));
        uart_command_pub_ = this->create_publisher<std_msgs::msg::Int16MultiArray>("uart_command", 10);
        aruco_enabled_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/aruco/enabled", rclcpp::QoS(1).reliable().transient_local());
        aruco_target_marker_pub_ = this->create_publisher<std_msgs::msg::Int32>("/aruco/target_marker_id", 10);
        yolo_enabled_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/yolo/enabled", rclcpp::QoS(1).reliable().transient_local());
        yolo_target_frame_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/yolo/target_frame", rclcpp::QoS(1).reliable().transient_local());
        spiral_monitor_timer_ = this->create_wall_timer(
            200ms, std::bind(&GPSWaypointFollower::monitorSpiralTargets, this));
        deactivateArucoTarget();
        deactivateYoloTarget();
        RCLCPP_INFO(this->get_logger(), "Waiting for initial GPS fix...");
    }

    void GPSWaypointFollower::onGpsFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        if (!gps_redy_ && msg->status.status >= 0) {
            ref_latitude_ = msg->latitude;
            ref_longitude_ = msg->longitude;
            gps_redy_ = true;
            RCLCPP_INFO(this->get_logger(), "Initial GPS fix acquired: lat=%.6f, lon=%.6f",
                        ref_latitude_, ref_longitude_);
            sendNextGoal();
        }
    }

    void GPSWaypointFollower::onMarkerDetected(const std_msgs::msg::Float32::SharedPtr msg) {
        latest_detected_marker_id_ = static_cast<int>(std::lround(msg->data));
        has_recent_aruco_detection_ = true;
        last_aruco_detection_time_ = this->now();

        if (!spiral_search_active_) {
            return;
        }
        
        // Check if current waypoint has a marker_id set
        if (goal_index_ >= waypoints_.size()) {
            return;
        }
        
        const auto& current_wp = waypoints_[goal_index_];
        float detected_id = msg->data;
        
        // Only interrupt if the detected marker ID matches the expected marker_id for current waypoint
        // marker_id == -1 means no marker is expected for this waypoint
        if (currentWaypointHasArucoTarget() &&
            std::abs(detected_id - static_cast<float>(current_wp.marker_id)) < 0.1f) {
            RCLCPP_INFO(this->get_logger(), "Marker ID %d detected during spiral search! Interrupting spiral search.", 
                       current_wp.marker_id);
            interruptSpiralSearch();
        }
    }

    void GPSWaypointFollower::interruptSpiralSearch() {
        if (!spiral_search_active_) {
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Interrupting spiral search. Canceling current spiral goal and handing off to ArUco approach.");
        cancelCurrentGoal();
        
        // Reset spiral search state
        spiral_search_active_ = false;
        spiral_index_ = 0;
        spiral_waypoints_.clear();

        // 正しいマーカーが見つかったので、ArUco 接近ノードへ制御を渡す
        waiting_for_aruco_goal_ = true;
        activateArucoTargetForCurrentWaypoint();
        RCLCPP_INFO(this->get_logger(), "Spiral search interrupted. Waiting for ArUco approach to finish.");
    }

    void GPSWaypointFollower::interruptSpiralSearchForYolo() {
        if (!spiral_search_active_) {
            return;
        }

        if (!currentWaypointHasYoloTarget()) {
            return;
        }

        RCLCPP_INFO(this->get_logger(),
                    "YOLO target '%s' detected during spiral search! Interrupting spiral search.",
                    waypoints_[goal_index_].yolo.c_str());
        cancelCurrentGoal();

        spiral_search_active_ = false;
        spiral_index_ = 0;
        spiral_waypoints_.clear();

        waiting_for_yolo_goal_ = true;
        activateYoloTargetForCurrentWaypoint();
        RCLCPP_INFO(this->get_logger(), "Spiral search interrupted. Waiting for YOLO approach to finish.");
    }

    void GPSWaypointFollower::restartSpiralSearchAfterTargetLost(const std::string& target_name) {
        if (goal_index_ >= waypoints_.size()) {
            return;
        }

        if (spiral_search_active_) {
            return;
        }

        waiting_for_aruco_goal_ = false;
        waiting_for_yolo_goal_ = false;
        spiral_index_ = 0;
        spiral_waypoints_.clear();

        if (currentWaypointHasArucoTarget()) {
            activateArucoTargetForCurrentWaypoint();
        }
        if (currentWaypointHasYoloTarget()) {
            activateYoloTargetForCurrentWaypoint();
        }

        RCLCPP_WARN(this->get_logger(),
                    "%s target approach failed or was canceled after TF was found. Restarting spiral search for waypoint %zu.",
                    target_name.c_str(), goal_index_ + 1);
        startSpiralSearch();
        sendNextGoal();
    }

    bool GPSWaypointFollower::currentWaypointHasArucoTarget() const {
        if (goal_index_ >= waypoints_.size()) {
            return false;
        }

        const auto& current_wp = waypoints_[goal_index_];
        return current_wp.aruco != "disable" && current_wp.marker_id >= 0;
    }

    bool GPSWaypointFollower::isCurrentArucoTargetVisible() const {
        if (!currentWaypointHasArucoTarget() || !has_recent_aruco_detection_) {
            return false;
        }

        const auto& current_wp = waypoints_[goal_index_];
        if (latest_detected_marker_id_ != current_wp.marker_id) {
            return false;
        }

        const double detection_age =
            (this->now() - last_aruco_detection_time_).seconds();
        if (detection_age > aruco_detection_max_age_sec_) {
            return false;
        }

        try {
            auto transform = tf_buffer_->lookupTransform(
                "map", "aruco_marker", tf2::TimePointZero);

            if (transform.header.stamp.sec != 0 || transform.header.stamp.nanosec != 0) {
                const double tf_age =
                    (this->now() - rclcpp::Time(transform.header.stamp)).seconds();
                if (tf_age > aruco_detection_max_age_sec_) {
                    return false;
                }
            }

            return true;
        } catch (const tf2::TransformException &) {
            return false;
        }
    }

    void GPSWaypointFollower::activateArucoTargetForCurrentWaypoint() {
        if (goal_index_ >= waypoints_.size()) {
            return;
        }

        const auto& current_wp = waypoints_[goal_index_];
        std_msgs::msg::Bool enabled_msg;
        enabled_msg.data = currentWaypointHasArucoTarget();
        aruco_enabled_pub_->publish(enabled_msg);

        std_msgs::msg::Int32 msg;
        msg.data = current_wp.marker_id;
        aruco_target_marker_pub_->publish(msg);
        aruco_target_active_ = enabled_msg.data;

        if (aruco_target_active_) {
            RCLCPP_INFO(this->get_logger(), "Activated ArUco target marker_id=%d for waypoint %zu",
                        current_wp.marker_id, goal_index_ + 1);
        }
    }

    void GPSWaypointFollower::deactivateArucoTarget() {
        std_msgs::msg::Bool enabled_msg;
        enabled_msg.data = false;
        aruco_enabled_pub_->publish(enabled_msg);

        std_msgs::msg::Int32 msg;
        msg.data = -1;
        aruco_target_marker_pub_->publish(msg);
        aruco_target_active_ = false;
    }

    bool GPSWaypointFollower::currentWaypointHasYoloTarget() const {
        if (goal_index_ >= waypoints_.size()) {
            return false;
        }

        const auto& yolo_target = waypoints_[goal_index_].yolo;
        return !yolo_target.empty() && yolo_target != "disable";
    }

    void GPSWaypointFollower::activateYoloTargetForCurrentWaypoint() {
        if (!currentWaypointHasYoloTarget()) {
            return;
        }

        std_msgs::msg::Bool enabled_msg;
        enabled_msg.data = true;
        yolo_enabled_pub_->publish(enabled_msg);

        std_msgs::msg::String msg;
        msg.data = waypoints_[goal_index_].yolo;
        yolo_target_frame_pub_->publish(msg);
        yolo_target_active_ = true;
        RCLCPP_INFO(this->get_logger(), "Activated YOLO target frame '%s' for waypoint %zu",
                    msg.data.c_str(), goal_index_ + 1);
    }

    void GPSWaypointFollower::deactivateYoloTarget() {
        std_msgs::msg::Bool enabled_msg;
        enabled_msg.data = false;
        yolo_enabled_pub_->publish(enabled_msg);

        std_msgs::msg::String msg;
        msg.data = "disable";
        yolo_target_frame_pub_->publish(msg);
        yolo_target_active_ = false;
    }

    void GPSWaypointFollower::onArucoGoalReached(const std_msgs::msg::Bool::SharedPtr msg) {
        if (!msg->data) {
            if (waiting_for_aruco_goal_ || aruco_target_active_) {
                restartSpiralSearchAfterTargetLost("ArUco");
            }
            return;
        }

        const bool should_handle =
            waiting_for_aruco_goal_ || spiral_search_active_ || aruco_target_active_;
        if (!should_handle) {
            return;
        }

        if (spiral_search_active_) {
            RCLCPP_INFO(this->get_logger(),
                        "ArUco goal reached while spiral search is active. Canceling spiral search and proceeding to next waypoint.");
            cancelCurrentGoal();
            spiral_search_active_ = false;
            spiral_index_ = 0;
            spiral_waypoints_.clear();
        } else {
            RCLCPP_INFO(this->get_logger(), "ArUco goal reached for waypoint %zu. Proceeding to next waypoint.",
                        goal_index_ + 1);
        }

        waiting_for_aruco_goal_ = false;
        deactivateArucoTarget();
        goal_index_++;
        sendNextGoal();
    }

    void GPSWaypointFollower::onYoloGoalReached(const std_msgs::msg::Bool::SharedPtr msg) {
        if (!msg->data) {
            if (waiting_for_yolo_goal_ || yolo_target_active_) {
                restartSpiralSearchAfterTargetLost("YOLO");
            }
            return;
        }

        const bool should_handle =
            waiting_for_yolo_goal_ || spiral_search_active_ || yolo_target_active_;
        if (!should_handle) {
            return;
        }

        if (spiral_search_active_) {
            RCLCPP_INFO(this->get_logger(),
                        "YOLO goal reached while spiral search is active. Canceling spiral search and proceeding to next waypoint.");
            cancelCurrentGoal();
            spiral_search_active_ = false;
            spiral_index_ = 0;
            spiral_waypoints_.clear();
        } else {
            RCLCPP_INFO(this->get_logger(), "YOLO goal reached for waypoint %zu. Proceeding to next waypoint.",
                        goal_index_ + 1);
        }

        waiting_for_yolo_goal_ = false;
        deactivateYoloTarget();
        goal_index_++;
        sendNextGoal();
    }

    void GPSWaypointFollower::cancelCurrentGoal() {
        std::lock_guard<std::mutex> lock(goal_handle_mutex_);
        if (current_goal_handle_) {
            auto status = current_goal_handle_->get_status();
            if (status == rclcpp_action::GoalStatus::STATUS_EXECUTING || 
                status == rclcpp_action::GoalStatus::STATUS_ACCEPTED) {
                RCLCPP_INFO(this->get_logger(), "Canceling current goal");
                action_client_->async_cancel_goal(current_goal_handle_);
            }
        }
    }

    geometry_msgs::msg::PoseStamped GPSWaypointFollower::makePoseStamped(double x, double y, double yaw) const {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = this->now();
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;

        auto q = geo::yaw_to_quaternion(yaw);
        pose.pose.orientation.x = q.x;
        pose.pose.orientation.y = q.y;
        pose.pose.orientation.z = q.z;
        pose.pose.orientation.w = q.w;
        return pose;
    }

    void GPSWaypointFollower::sendNextGoal() {
        if (!gps_redy_) {
            RCLCPP_INFO(this->get_logger(), "Waiting for GPS fix...");
            return;
        }
        if (waiting_for_aruco_goal_ || waiting_for_yolo_goal_) {
            RCLCPP_INFO(this->get_logger(), "Waiting for external target approach before sending the next GPS goal.");
            return;
        }
        if (spiral_search_active_) {
            if (spiral_index_ >= spiral_waypoints_.size()) {
                RCLCPP_INFO(this->get_logger(), "Spiral search finished without finding the requested target.");
                spiral_search_active_ = false;
                spiral_index_ = 0;
                spiral_waypoints_.clear();

                if (aruco_target_active_) {
                    RCLCPP_WARN(this->get_logger(),
                                "ArUco marker_id=%d was not found during spiral search. Proceeding to the next waypoint.",
                                waypoints_[goal_index_].marker_id);
                    waiting_for_aruco_goal_ = false;
                    deactivateArucoTarget();
                }
                if (currentWaypointHasYoloTarget()) {
                    RCLCPP_WARN(this->get_logger(),
                                "YOLO target '%s' was not found during spiral search. Proceeding to the next waypoint.",
                                waypoints_[goal_index_].yolo.c_str());
                    waiting_for_yolo_goal_ = false;
                    deactivateYoloTarget();
                }

                goal_index_++;
                sendNextGoal();
                return;
            }
            const auto& wp = spiral_waypoints_[spiral_index_];
            auto pose = makePoseStamped(wp.x, wp.y, wp.yaw);

            NavigateToPose::Goal goal;
            goal.pose = pose;

            {
                std::ostringstream oss;
                oss << "Sending spiral goal " << spiral_index_
                    << ": x=" << wp.x << ", y=" << wp.y << ", yaw=" << wp.yaw;
                last_sent_goal_log_ = oss.str();
            }
            RCLCPP_INFO(this->get_logger(), "%s", last_sent_goal_log_.c_str());

            if (!action_client_->wait_for_action_server(2s)) {
                RCLCPP_WARN(this->get_logger(), "Action server not available after waiting");
                return;
            }

            rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;
            opts.goal_response_callback = [this](GoalHandleNavigateToPose::SharedPtr handle) {
                if (!handle) {
                    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
                } else {
                    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
                    std::lock_guard<std::mutex> lock(goal_handle_mutex_);
                    current_goal_handle_ = handle;
                }
            };
            opts.result_callback = std::bind(&GPSWaypointFollower::onResult, this, std::placeholders::_1);
            action_client_->async_send_goal(goal, opts);
            return;
        }
        // Gps waypoint navigation
        if (goal_index_ >= waypoints_.size()) {
            RCLCPP_INFO(this->get_logger(), "All waypoints visited");
            deactivateArucoTarget();
            deactivateYoloTarget();
            return;
        }

        const auto& wp = waypoints_[goal_index_];
        auto enu = geo::latlon_to_enu(wp.latitude, wp.longitude, ref_latitude_, ref_longitude_);
        auto pose = makePoseStamped(enu.first, enu.second, wp.yaw);

        NavigateToPose::Goal goal;
        goal.pose = pose;

        {
            std::ostringstream oss;
            oss << "Sending goal " << (goal_index_ + 1)
                << ": x=" << enu.first
                << ", y=" << enu.second
                << " : GNSS " << wp.latitude
                << ", " << wp.longitude
                << ", yaw=" << wp.yaw;
            last_sent_goal_log_ = oss.str();
        }
        RCLCPP_INFO(this->get_logger(), "%s", last_sent_goal_log_.c_str());

        if (!action_client_->wait_for_action_server(2s)) {
            RCLCPP_WARN(this->get_logger(), "Action server not available");
            return;
        }

        rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;
        // 修正: GoalResponseCallback は void(GoalHandle::SharedPtr) を要求するため、ラムダで適合させる
        opts.goal_response_callback = [this](GoalHandleNavigateToPose::SharedPtr handle) {
            if (!handle) {
                RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            } else {
                RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
                std::lock_guard<std::mutex> lock(goal_handle_mutex_);
                current_goal_handle_ = handle;
            }
        };
        opts.result_callback = std::bind(&GPSWaypointFollower::onResult, this, std::placeholders::_1);
        action_client_->async_send_goal(goal, opts);
    }

    void GPSWaypointFollower::onGoalResponse(std::shared_future<GoalHandleNavigateToPose::SharedPtr> future) {
        auto handle = future.get();
        if (!handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    bool GPSWaypointFollower::shouldStartSpiralSearch() const {
        if (goal_index_ >= waypoints_.size()) return false;
        return waypoints_[goal_index_].spiral_search;
    }
    void GPSWaypointFollower::startSpiralSearch() {
        const auto& wp = waypoints_[goal_index_];
        auto base = geo::latlon_to_enu(wp.latitude, wp.longitude, ref_latitude_, ref_longitude_);

        spiral_waypoints_.clear();
        spiral_waypoints_.reserve(static_cast<size_t>(spiral_params_.n_spiral));

        for (int i = 0; i < spiral_params_.n_spiral; ++i) {
            double r = spiral_params_.r0 + static_cast<double>(i) * spiral_params_.dr;
            double theta = static_cast<double>(i) * spiral_params_.dtheta_rad;
            LocalPose2D p;
            p.x = base.first + r * std::cos(theta);
            p.y = base.second + r * std::sin(theta);
            p.yaw = wp.yaw; // keep the same yaw as the waypoint
            spiral_waypoints_.push_back(p);
        }
        spiral_index_ = 0;
        spiral_search_active_ = true;
        RCLCPP_INFO(this->get_logger(), "Starting spiral search with %zu waypoints",
                    spiral_waypoints_.size());
    }

    void GPSWaypointFollower::monitorSpiralTargets() {
        if (!spiral_search_active_ || goal_index_ >= waypoints_.size()) {
            return;
        }

        if (currentWaypointHasArucoTarget() && isCurrentArucoTargetVisible()) {
            RCLCPP_INFO(this->get_logger(),
                        "ArUco marker_id=%d became visible during spiral search. Switching from spiral goal to ArUco goal.",
                        waypoints_[goal_index_].marker_id);
            interruptSpiralSearch();
            return;
        }

        if (currentWaypointHasYoloTarget() && isCurrentYoloTargetVisible()) {
            RCLCPP_INFO(this->get_logger(),
                        "YOLO target '%s' became visible during spiral search. Switching from spiral goal to YOLO goal.",
                        waypoints_[goal_index_].yolo.c_str());
            interruptSpiralSearchForYolo();
        }
    }

    bool GPSWaypointFollower::isCurrentYoloTargetVisible() const {
        if (!currentWaypointHasYoloTarget()) {
            return false;
        }

        try {
            const auto& target_frame = waypoints_[goal_index_].yolo;
            auto transform = tf_buffer_->lookupTransform(
                "map", target_frame, tf2::TimePointZero);

            const bool has_stamp =
                transform.header.stamp.sec != 0 || transform.header.stamp.nanosec != 0;
            if (yolo_tf_max_age_sec_ > 0.0 && has_stamp) {
                const double tf_age = (this->now() - rclcpp::Time(transform.header.stamp)).seconds();
                if (tf_age > yolo_tf_max_age_sec_) {
                    return false;
                }
            }

            return true;
        } catch (const tf2::TransformException &) {
            return false;
        }
    }

    void GPSWaypointFollower::onResult(const GoalHandleNavigateToPose::WrappedResult &result) {
        // Clear goal handle
        {
            std::lock_guard<std::mutex> lock(goal_handle_mutex_);
            current_goal_handle_.reset();
        }

        if (result.code == rclcpp_action::ResultCode::CANCELED) {
            RCLCPP_INFO(this->get_logger(), "Goal was canceled");
            return;
        }

        if (result.code == rclcpp_action::ResultCode::ABORTED) {
            RCLCPP_WARN(this->get_logger(), "Goal was aborted");
            return;
        }

        if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_WARN(this->get_logger(), "Goal finished with unexpected result code: %d",
                        static_cast<int>(result.code));
            return;
        }

        {
            RCLCPP_INFO(this->get_logger(), "Reached goal");
            if (!last_sent_goal_log_.empty()) {
                RCLCPP_INFO(this->get_logger(), "\033[32mReached published goal: %s\033[0m",
                            last_sent_goal_log_.c_str());
            }
            // uart_command に 0x441 を3回送信（serial_publiasher が UART で MCU に送る）
            std_msgs::msg::Int16MultiArray uart_msg;
            uart_msg.data = {static_cast<int16_t>(0x441), 0, static_cast<int16_t>(0x441), 0};
            for (int i = 0; i < 3; ++i) {
                uart_command_pub_->publish(uart_msg);
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            std::this_thread::sleep_for(std::chrono::duration<double>(hold_time_sec_));
        }

        if (spiral_search_active_) {
            spiral_index_ ++;
            sendNextGoal();
            return;
        }

        if (goal_index_ < waypoints_.size() && currentWaypointHasArucoTarget()) {
            activateArucoTargetForCurrentWaypoint();
            if (isCurrentArucoTargetVisible()) {
                waiting_for_aruco_goal_ = true;
                RCLCPP_INFO(this->get_logger(),
                            "Reached GPS waypoint %zu. ArUco marker_id=%d is already visible, skipping spiral search.",
                            goal_index_ + 1, waypoints_[goal_index_].marker_id);
            } else if (shouldStartSpiralSearch()) {
                startSpiralSearch();
                sendNextGoal();
            } else {
                waiting_for_aruco_goal_ = true;
                RCLCPP_INFO(this->get_logger(), "Reached GPS waypoint %zu. Waiting for marker_id=%d approach.",
                            goal_index_ + 1, waypoints_[goal_index_].marker_id);
            }
            return;
        }

        if (goal_index_ < waypoints_.size() && currentWaypointHasYoloTarget()) {
            activateYoloTargetForCurrentWaypoint();
            if (isCurrentYoloTargetVisible()) {
                waiting_for_yolo_goal_ = true;
                RCLCPP_INFO(this->get_logger(),
                            "Reached GPS waypoint %zu. YOLO target '%s' is already visible, skipping spiral search.",
                            goal_index_ + 1, waypoints_[goal_index_].yolo.c_str());
                return;
            }
        }

        if (shouldStartSpiralSearch()) {
            if (goal_index_ < waypoints_.size() && currentWaypointHasYoloTarget()) {
                RCLCPP_INFO(this->get_logger(), "Reached GPS waypoint %zu. Starting spiral search before YOLO approach.",
                            goal_index_ + 1);
            }
            startSpiralSearch();
            sendNextGoal();
            return;
        }

        if (goal_index_ < waypoints_.size() && currentWaypointHasYoloTarget()) {
            activateYoloTargetForCurrentWaypoint();
            waiting_for_yolo_goal_ = true;
            RCLCPP_INFO(this->get_logger(), "Reached GPS waypoint %zu. Waiting for YOLO target '%s'.",
                        goal_index_ + 1, waypoints_[goal_index_].yolo.c_str());
            return;
        }

        {
            goal_index_ ++;
            sendNextGoal();
        }
    }

}
