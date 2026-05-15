//
// Created by karisora on 2025/09/10.
//

#ifndef ARES_NAV2_GPS_WAYPOINT_FOLLOWER_HPP
#define ARES_NAV2_GPS_WAYPOINT_FOLLOWER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <fstream>
#include <cmath>
#include <vector>
#include <string>
#include <optional>
#include <mutex>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


namespace ares_nav2
{

struct GPSWaypoint
{
    double latitude{};
    double longitude{};
    double yaw{}; // in radians
    bool spiral_search{false};
    std::string aruco{"disable"}; // disable | enable
    int marker_id{-1}; // marker ID to detect during spiral search (-1 means no marker)
    std::string yolo{"disable"}; // disable | bottle | mallet ...
};

struct LocalPose2D
{
    double x{};
    double y{};
    double yaw{}; // in radians
};

struct SpiralParams {
    double r0{1.0};
    double dr{1.0};
    double dtheta_rad{M_PI / 2.0};
    int n_spiral{7};
};

class GPSWaypointFollower : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    explicit GPSWaypointFollower(
        const std::string& waypoint_yaml_path,
        const SpiralParams& spiral_params = {});


private:
    enum class MissionLogLevel {
        Info,
        Warn,
        Error
    };

    // ROS2 interfaces
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr marker_detected_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr aruco_goal_reached_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr yolo_goal_reached_sub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr uart_command_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr aruco_enabled_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr aruco_target_marker_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr yolo_enabled_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr yolo_target_frame_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_status_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr spiral_monitor_timer_;
    rclcpp::TimerBase::SharedPtr mission_status_timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Waypoint management/state
    std::vector<GPSWaypoint> waypoints_;
    size_t goal_index_{0};
    double ref_latitude_{0.0};
    double ref_longitude_{0.0};
    bool gps_redy_{false};
    double hold_time_sec_{3.0};
    bool waiting_for_aruco_goal_{false};
    bool waiting_for_yolo_goal_{false};
    bool aruco_target_active_{false};
    bool yolo_target_active_{false};
    double aruco_detection_max_age_sec_{1.0};
    double yolo_tf_max_age_sec_{0.5};
    int latest_detected_marker_id_{-1};
    bool has_recent_aruco_detection_{false};
    rclcpp::Time last_aruco_detection_time_{0, 0, RCL_ROS_TIME};
    bool mission_log_to_file_{false};
    std::string mission_log_directory_{"mission_logs"};
    std::string mission_status_topic_{"/mission/status"};
    double mission_status_period_sec_{1.0};
    std::string mission_log_path_;
    std::ofstream mission_log_file_;
    std::mutex mission_log_mutex_;
    std::string last_mission_phase_{"INIT"};
    std::string last_mission_message_{"Starting mission node."};
    std::string last_mission_level_{"info"};
    std::string last_mission_result_{"running"};
    std::string active_goal_description_{"none"};
    rclcpp::Time active_goal_start_time_{0, 0, RCL_ROS_TIME};
    bool has_active_goal_start_time_{false};

    // Spiral search parameters
    bool spiral_search_active_{false};
    SpiralParams spiral_params_;
    std::vector<LocalPose2D> spiral_waypoints_;
    size_t spiral_index_{0};
    size_t spiral_count_waypoint_index_{static_cast<size_t>(-1)};
    int spiral_attempt_count_{0};
    bool spiral_spin_scan_active_{false};
    bool spiral_spin_scan_enabled_{true};
    double spiral_spin_scan_total_angle_rad_{2.0 * M_PI};
    double spiral_spin_scan_angular_speed_rad_s_{0.5};
    double spiral_spin_scan_linear_speed_m_s_{0.0};
    int spiral_spin_scan_direction_{1};
    rclcpp::Time spiral_spin_scan_start_time_{0, 0, RCL_ROS_TIME};

    // Goal handle management
    std::mutex goal_handle_mutex_;
    GoalHandleNavigateToPose::SharedPtr current_goal_handle_;
    std::string last_sent_goal_log_;

    // Callbacks
    void onGpsFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void onMarkerDetected(const std_msgs::msg::Float32::SharedPtr msg);
    void onArucoGoalReached(const std_msgs::msg::Bool::SharedPtr msg);
    void onYoloGoalReached(const std_msgs::msg::Bool::SharedPtr msg);
    void sendNextGoal();
    void startSpiralSearch();
    bool shouldStartSpiralSearch() const;
    void cancelCurrentGoal();
    void interruptSpiralSearch();
    void interruptSpiralSearchForYolo();
    void restartSpiralSearchAfterTargetLost(const std::string& target_name);
    void activateArucoTargetForCurrentWaypoint();
    void deactivateArucoTarget();
    bool currentWaypointHasArucoTarget() const;
    bool isCurrentArucoTargetVisible() const;
    void activateYoloTargetForCurrentWaypoint();
    void deactivateYoloTarget();
    bool currentWaypointHasYoloTarget() const;
    void monitorSpiralTargets();
    bool isCurrentYoloTargetVisible() const;
    bool shouldSpinScanAtSpiralPoint() const;
    double spiralSpinScanDurationSec() const;
    void startSpiralSpinScan();
    void updateSpiralSpinScan();
    void stopSpiralSpinScan();
    void publishZeroCmdVel();
    void publishGoalReachedUartCommand(const std::string& goal_type);

    // Utility functions
    void onGoalResponse(std::shared_future<GoalHandleNavigateToPose::SharedPtr> future);
    void onResult(const GoalHandleNavigateToPose::WrappedResult& result);

    // helpers
    geometry_msgs::msg::PoseStamped makePoseStamped(double x, double y, double yaw) const;
    void openMissionLogFile();
    void missionLog(
        MissionLogLevel level,
        const std::string& phase,
        const std::string& message);
    void publishMissionStatus();
    void setActiveGoalDescription(const std::string& description);
    std::string missionLevelString(MissionLogLevel level) const;
    std::string missionResultString(MissionLogLevel level, const std::string& phase) const;
    double activeGoalElapsedSec() const;
    std::string missionContext() const;
    std::string wallTimeString(const char* format) const;
};
} // namespace ares_nav2

#endif //ARES_NAV2_GPS_WAYPOINT_FOLLOWER_HPP
