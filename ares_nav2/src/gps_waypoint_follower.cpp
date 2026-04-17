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
#include <ctime>
#include <filesystem>
#include <iomanip>
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
	        mission_log_to_file_ = this->declare_parameter<bool>("mission_log_to_file", true);
	        mission_log_directory_ = this->declare_parameter<std::string>(
	            "mission_log_directory", "mission_logs");
	        openMissionLogFile();
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
	        {
	            std::ostringstream oss;
	            oss << "Loaded " << waypoints_.size()
	                << " waypoint(s). Waiting for initial GPS fix.";
	            missionLog(MissionLogLevel::Info, "INIT", oss.str());
	        }
	    }

	    std::string GPSWaypointFollower::wallTimeString(const char* format) const {
	        const auto now = std::chrono::system_clock::now();
	        const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
	        std::tm local_time{};
	        localtime_r(&now_time, &local_time);

	        std::ostringstream oss;
	        oss << std::put_time(&local_time, format);
	        return oss.str();
	    }

	    void GPSWaypointFollower::openMissionLogFile() {
	        if (!mission_log_to_file_) {
	            return;
	        }

	        try {
	            std::filesystem::path log_dir(mission_log_directory_);
	            if (log_dir.is_relative()) {
	                log_dir = std::filesystem::current_path() / log_dir;
	            }
	            std::filesystem::create_directories(log_dir);

	            const std::string filename =
	                "mission_" + wallTimeString("%Y%m%d_%H%M%S") + ".log";
	            const auto log_path = log_dir / filename;
	            mission_log_path_ = log_path.string();
	            mission_log_file_.open(mission_log_path_, std::ios::out | std::ios::app);
	            if (!mission_log_file_.is_open()) {
	                RCLCPP_WARN(this->get_logger(), "Failed to open mission log file: %s",
	                            mission_log_path_.c_str());
	                return;
	            }
	            RCLCPP_INFO(this->get_logger(), "Mission log file: %s", mission_log_path_.c_str());
	        } catch (const std::exception& ex) {
	            RCLCPP_WARN(this->get_logger(), "Failed to prepare mission log file: %s", ex.what());
	        }
	    }

	    std::string GPSWaypointFollower::missionContext() const {
	        std::ostringstream oss;
	        oss << "wp=";
	        if (goal_index_ < waypoints_.size()) {
	            const auto& wp = waypoints_[goal_index_];
	            oss << (goal_index_ + 1) << "/" << waypoints_.size()
	                << " gps=(" << wp.latitude << "," << wp.longitude << ")"
	                << " aruco=" << wp.aruco
	                << " marker_id=" << wp.marker_id
	                << " yolo=" << wp.yolo
	                << " spiral_enabled=" << (wp.spiral_search ? "true" : "false");
	        } else {
	            oss << "done/" << waypoints_.size();
	        }

	        if (spiral_search_active_) {
	            oss << " spiral_step=" << (spiral_index_ + 1) << "/"
	                << spiral_waypoints_.size();
	        } else {
	            oss << " spiral_step=inactive";
	        }
	        oss << " spiral_attempt=" << spiral_attempt_count_;
	        return oss.str();
	    }

	    void GPSWaypointFollower::missionLog(
	        MissionLogLevel level,
	        const std::string& phase,
	        const std::string& message) {
	        std::ostringstream oss;
	        oss << "[MISSION]"
	            << " local_time=" << wallTimeString("%Y-%m-%d %H:%M:%S")
	            << " phase=" << phase
	            << " " << missionContext()
	            << " | " << message;
	        const std::string line = oss.str();
	        constexpr const char* green = "\033[32m";
	        constexpr const char* yellow = "\033[33m";
	        constexpr const char* red = "\033[31m";
	        constexpr const char* reset = "\033[0m";
	        const bool is_goal_reached =
	            phase.find("REACHED") != std::string::npos || phase == "MISSION_DONE";
	        const char* color = "";
	        if (level == MissionLogLevel::Error) {
	            color = red;
	        } else if (level == MissionLogLevel::Warn) {
	            color = yellow;
	        } else if (is_goal_reached) {
	            color = green;
	        }
	        const std::string console_line =
	            color[0] == '\0' ? line : std::string(color) + line + reset;

	        switch (level) {
	            case MissionLogLevel::Warn:
	                RCLCPP_WARN(this->get_logger(), "%s", console_line.c_str());
	                break;
	            case MissionLogLevel::Error:
	                RCLCPP_ERROR(this->get_logger(), "%s", console_line.c_str());
	                break;
	            case MissionLogLevel::Info:
	            default:
	                RCLCPP_INFO(this->get_logger(), "%s", console_line.c_str());
	                break;
	        }

	        std::lock_guard<std::mutex> lock(mission_log_mutex_);
	        if (mission_log_file_.is_open()) {
	            mission_log_file_ << line << std::endl;
	        }
	    }

	    void GPSWaypointFollower::onGpsFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
	        if (!gps_redy_ && msg->status.status >= 0) {
	            ref_latitude_ = msg->latitude;
	            ref_longitude_ = msg->longitude;
	            gps_redy_ = true;
	            {
	                std::ostringstream oss;
	                oss << "Initial GPS fix acquired: lat=" << ref_latitude_
	                    << ", lon=" << ref_longitude_;
	                missionLog(MissionLogLevel::Info, "GPS_READY", oss.str());
	            }
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
	            {
	                std::ostringstream oss;
	                oss << "Marker ID " << current_wp.marker_id
	                    << " detected during spiral search. Interrupting spiral search.";
	                missionLog(MissionLogLevel::Info, "TARGET_FOUND_ARUCO", oss.str());
	            }
	            interruptSpiralSearch();
	        }
	    }

    void GPSWaypointFollower::interruptSpiralSearch() {
        if (!spiral_search_active_) {
            return;
        }

	        missionLog(
	            MissionLogLevel::Info,
	            "HANDOFF_ARUCO",
	            "Canceling current spiral goal and handing off to ArUco approach.");
        cancelCurrentGoal();

        // Reset spiral search state
        spiral_search_active_ = false;
        spiral_index_ = 0;
        spiral_waypoints_.clear();

        // 正しいマーカーが見つかったので、ArUco 接近ノードへ制御を渡す
	        waiting_for_aruco_goal_ = true;
	        activateArucoTargetForCurrentWaypoint();
	        missionLog(MissionLogLevel::Info, "WAIT_ARUCO_APPROACH",
	                   "Spiral search interrupted. Waiting for ArUco approach to finish.");
    }

    void GPSWaypointFollower::interruptSpiralSearchForYolo() {
        if (!spiral_search_active_) {
            return;
        }

        if (!currentWaypointHasYoloTarget()) {
            return;
        }

	        {
	            std::ostringstream oss;
	            oss << "YOLO target '" << waypoints_[goal_index_].yolo
	                << "' detected during spiral search. Interrupting spiral search.";
	            missionLog(MissionLogLevel::Info, "TARGET_FOUND_YOLO", oss.str());
	        }
        cancelCurrentGoal();

        spiral_search_active_ = false;
        spiral_index_ = 0;
        spiral_waypoints_.clear();

	        waiting_for_yolo_goal_ = true;
	        activateYoloTargetForCurrentWaypoint();
	        missionLog(MissionLogLevel::Info, "WAIT_YOLO_APPROACH",
	                   "Spiral search interrupted. Waiting for YOLO approach to finish.");
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

	        {
	            std::ostringstream oss;
	            oss << target_name
	                << " target approach failed or was canceled after TF was found. "
	                << "Restarting spiral search for this waypoint.";
	            missionLog(MissionLogLevel::Warn, "TARGET_LOST_RETRY_SPIRAL", oss.str());
	        }
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
	            std::ostringstream oss;
	            oss << "Activated ArUco target marker_id=" << current_wp.marker_id;
	            missionLog(MissionLogLevel::Info, "TARGET_ACTIVE_ARUCO", oss.str());
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
	        {
	            std::ostringstream oss;
	            oss << "Activated YOLO target frame '" << msg.data << "'";
	            missionLog(MissionLogLevel::Info, "TARGET_ACTIVE_YOLO", oss.str());
	        }
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
	            missionLog(
	                MissionLogLevel::Info,
	                "ARUCO_REACHED",
	                "ArUco goal reached while spiral search is active. Canceling spiral search and proceeding to next waypoint.");
	            cancelCurrentGoal();
	            spiral_search_active_ = false;
	            spiral_index_ = 0;
	            spiral_waypoints_.clear();
	        } else {
	            missionLog(MissionLogLevel::Info, "ARUCO_REACHED",
	                       "ArUco goal reached. Proceeding to next waypoint.");
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
	            missionLog(
	                MissionLogLevel::Info,
	                "YOLO_REACHED",
	                "YOLO goal reached while spiral search is active. Canceling spiral search and proceeding to next waypoint.");
	            cancelCurrentGoal();
	            spiral_search_active_ = false;
	            spiral_index_ = 0;
	            spiral_waypoints_.clear();
	        } else {
	            missionLog(MissionLogLevel::Info, "YOLO_REACHED",
	                       "YOLO goal reached. Proceeding to next waypoint.");
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
	            missionLog(MissionLogLevel::Info, "WAIT_GPS", "Waiting for GPS fix.");
	            return;
	        }
	        if (waiting_for_aruco_goal_ || waiting_for_yolo_goal_) {
	            missionLog(
	                MissionLogLevel::Info,
	                "WAIT_TARGET_APPROACH",
	                "Waiting for external target approach before sending the next GPS goal.");
	            return;
	        }
	        if (spiral_search_active_) {
	            if (spiral_index_ >= spiral_waypoints_.size()) {
	                missionLog(
	                    MissionLogLevel::Warn,
	                    "SPIRAL_FINISHED_NOT_FOUND",
	                    "Spiral search finished without finding the requested target.");
	                spiral_search_active_ = false;
	                spiral_index_ = 0;
	                spiral_waypoints_.clear();

	                if (aruco_target_active_) {
	                    std::ostringstream oss;
	                    oss << "ArUco marker_id=" << waypoints_[goal_index_].marker_id
	                        << " was not found during spiral search. Proceeding to the next waypoint.";
	                    missionLog(MissionLogLevel::Warn, "ARUCO_NOT_FOUND_SKIP", oss.str());
	                    waiting_for_aruco_goal_ = false;
	                    deactivateArucoTarget();
	                }
	                if (currentWaypointHasYoloTarget()) {
	                    std::ostringstream oss;
	                    oss << "YOLO target '" << waypoints_[goal_index_].yolo
	                        << "' was not found during spiral search. Proceeding to the next waypoint.";
	                    missionLog(MissionLogLevel::Warn, "YOLO_NOT_FOUND_SKIP", oss.str());
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
	                oss << "Sending spiral goal " << (spiral_index_ + 1)
	                    << "/" << spiral_waypoints_.size()
	                    << " attempt=" << spiral_attempt_count_
	                    << ": x=" << wp.x << ", y=" << wp.y << ", yaw=" << wp.yaw;
	                last_sent_goal_log_ = oss.str();
	            }
	            missionLog(MissionLogLevel::Info, "SPIRAL_GOAL", last_sent_goal_log_);

	            if (!action_client_->wait_for_action_server(2s)) {
	                missionLog(MissionLogLevel::Warn, "NAV2_UNAVAILABLE",
	                           "Action server not available after waiting.");
	                return;
	            }

            rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;
	            opts.goal_response_callback = [this](GoalHandleNavigateToPose::SharedPtr handle) {
	                if (!handle) {
	                    missionLog(MissionLogLevel::Error, "NAV2_REJECTED",
	                               "Goal was rejected by server.");
	                } else {
	                    missionLog(MissionLogLevel::Info, "NAV2_ACCEPTED",
	                               "Goal accepted by server, waiting for result.");
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
	            missionLog(MissionLogLevel::Info, "MISSION_DONE", "All waypoints visited.");
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
	            oss << "Sending GPS goal " << (goal_index_ + 1) << "/" << waypoints_.size()
	                << ": x=" << enu.first
	                << ", y=" << enu.second
	                << " : GNSS " << wp.latitude
                << ", " << wp.longitude
                << ", yaw=" << wp.yaw;
	            last_sent_goal_log_ = oss.str();
	        }
	        missionLog(MissionLogLevel::Info, "GPS_GOAL", last_sent_goal_log_);

	        if (!action_client_->wait_for_action_server(2s)) {
	            missionLog(MissionLogLevel::Warn, "NAV2_UNAVAILABLE", "Action server not available.");
	            return;
	        }

        rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;
        // 修正: GoalResponseCallback は void(GoalHandle::SharedPtr) を要求するため、ラムダで適合させる
	        opts.goal_response_callback = [this](GoalHandleNavigateToPose::SharedPtr handle) {
	            if (!handle) {
	                missionLog(MissionLogLevel::Error, "NAV2_REJECTED",
	                           "Goal was rejected by server.");
	            } else {
	                missionLog(MissionLogLevel::Info, "NAV2_ACCEPTED",
	                           "Goal accepted by server, waiting for result.");
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
	            missionLog(MissionLogLevel::Error, "NAV2_REJECTED",
	                       "Goal was rejected by server.");
	        } else {
	            missionLog(MissionLogLevel::Info, "NAV2_ACCEPTED",
	                       "Goal accepted by server, waiting for result.");
	        }
	    }

    bool GPSWaypointFollower::shouldStartSpiralSearch() const {
        if (goal_index_ >= waypoints_.size()) return false;
        return waypoints_[goal_index_].spiral_search;
    }
	    void GPSWaypointFollower::startSpiralSearch() {
	        const auto& wp = waypoints_[goal_index_];
	        auto base = geo::latlon_to_enu(wp.latitude, wp.longitude, ref_latitude_, ref_longitude_);
	        if (spiral_count_waypoint_index_ != goal_index_) {
	            spiral_count_waypoint_index_ = goal_index_;
	            spiral_attempt_count_ = 0;
	        }
	        ++spiral_attempt_count_;

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
	        {
	            std::ostringstream oss;
	            oss << "Starting spiral search attempt " << spiral_attempt_count_
	                << " with " << spiral_waypoints_.size()
	                << " spiral waypoint(s).";
	            missionLog(MissionLogLevel::Info, "SPIRAL_START", oss.str());
	        }
	    }

    void GPSWaypointFollower::monitorSpiralTargets() {
        if (!spiral_search_active_ || goal_index_ >= waypoints_.size()) {
            return;
        }

	        if (currentWaypointHasArucoTarget() && isCurrentArucoTargetVisible()) {
	            {
	                std::ostringstream oss;
	                oss << "ArUco marker_id=" << waypoints_[goal_index_].marker_id
	                    << " became visible during spiral search. Switching from spiral goal to ArUco goal.";
	                missionLog(MissionLogLevel::Info, "TARGET_VISIBLE_ARUCO", oss.str());
	            }
	            interruptSpiralSearch();
	            return;
	        }

	        if (currentWaypointHasYoloTarget() && isCurrentYoloTargetVisible()) {
	            {
	                std::ostringstream oss;
	                oss << "YOLO target '" << waypoints_[goal_index_].yolo
	                    << "' became visible during spiral search. Switching from spiral goal to YOLO goal.";
	                missionLog(MissionLogLevel::Info, "TARGET_VISIBLE_YOLO", oss.str());
	            }
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
	            missionLog(MissionLogLevel::Warn, "NAV2_CANCELED", "Goal was canceled.");
	            return;
	        }

	        if (result.code == rclcpp_action::ResultCode::ABORTED) {
	            missionLog(MissionLogLevel::Warn, "NAV2_ABORTED", "Goal was aborted.");
	            return;
	        }

	        if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
	            std::ostringstream oss;
	            oss << "Goal finished with unexpected result code: "
	                << static_cast<int>(result.code);
	            missionLog(MissionLogLevel::Warn, "NAV2_UNEXPECTED_RESULT", oss.str());
	            return;
	        }

	        {
	            if (!last_sent_goal_log_.empty()) {
	                missionLog(MissionLogLevel::Info, "NAV2_REACHED",
	                           "Reached published goal: " + last_sent_goal_log_);
	            } else {
	                missionLog(MissionLogLevel::Info, "NAV2_REACHED", "Reached goal.");
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
	            {
	                std::ostringstream oss;
	                oss << "Reached spiral step " << (spiral_index_ + 1)
	                    << "/" << spiral_waypoints_.size()
	                    << " on attempt " << spiral_attempt_count_ << ".";
	                missionLog(MissionLogLevel::Info, "SPIRAL_STEP_REACHED", oss.str());
	            }
	            spiral_index_ ++;
	            sendNextGoal();
	            return;
        }

        if (goal_index_ < waypoints_.size() && currentWaypointHasArucoTarget()) {
	            activateArucoTargetForCurrentWaypoint();
	            if (isCurrentArucoTargetVisible()) {
	                waiting_for_aruco_goal_ = true;
	                {
	                    std::ostringstream oss;
	                    oss << "Reached GPS waypoint. ArUco marker_id="
	                        << waypoints_[goal_index_].marker_id
	                        << " is already visible, skipping spiral search.";
	                    missionLog(MissionLogLevel::Info, "WAIT_ARUCO_APPROACH", oss.str());
	                }
	            } else if (shouldStartSpiralSearch()) {
	                startSpiralSearch();
	                sendNextGoal();
	            } else {
	                waiting_for_aruco_goal_ = true;
	                {
	                    std::ostringstream oss;
	                    oss << "Reached GPS waypoint. Waiting for ArUco marker_id="
	                        << waypoints_[goal_index_].marker_id << " approach.";
	                    missionLog(MissionLogLevel::Info, "WAIT_ARUCO_APPROACH", oss.str());
	                }
	            }
	            return;
	        }

        if (goal_index_ < waypoints_.size() && currentWaypointHasYoloTarget()) {
            activateYoloTargetForCurrentWaypoint();
	            if (isCurrentYoloTargetVisible()) {
	                waiting_for_yolo_goal_ = true;
	                {
	                    std::ostringstream oss;
	                    oss << "Reached GPS waypoint. YOLO target '"
	                        << waypoints_[goal_index_].yolo
	                        << "' is already visible, skipping spiral search.";
	                    missionLog(MissionLogLevel::Info, "WAIT_YOLO_APPROACH", oss.str());
	                }
	                return;
	            }
	        }

	        if (shouldStartSpiralSearch()) {
	            if (goal_index_ < waypoints_.size() && currentWaypointHasYoloTarget()) {
	                missionLog(MissionLogLevel::Info, "GPS_REACHED_START_SPIRAL",
	                           "Reached GPS waypoint. Starting spiral search before YOLO approach.");
	            }
	            startSpiralSearch();
	            sendNextGoal();
            return;
        }

	        if (goal_index_ < waypoints_.size() && currentWaypointHasYoloTarget()) {
	            activateYoloTargetForCurrentWaypoint();
	            waiting_for_yolo_goal_ = true;
	            {
	                std::ostringstream oss;
	                oss << "Reached GPS waypoint. Waiting for YOLO target '"
	                    << waypoints_[goal_index_].yolo << "'.";
	                missionLog(MissionLogLevel::Info, "WAIT_YOLO_APPROACH", oss.str());
	            }
	            return;
	        }

	        {
	            missionLog(MissionLogLevel::Info, "WAYPOINT_DONE",
	                       "Waypoint has no active target handling. Proceeding to next waypoint.");
	            goal_index_ ++;
	            sendNextGoal();
	        }
    }

}
