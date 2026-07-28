// ERC Phase 1b — prior-map relocalization ("map anchor").
//
// Keeps the verified Ericsii FAST-LIO as the local odometry authority
// (it publishes camera_init -> body). This node adds the *global* correction:
// it GICP-aligns the live FAST-LIO cloud (/cloud_registered, expressed in the
// camera_init/odom frame) onto a saved prior map (.pcd, expressed in the map
// frame) and broadcasts a low-passed  map -> camera_init  transform.
//
// A second, independent correction source can feed the SAME transform: once
// aruco_map_anchor.py has frozen map->datum (see its module docstring), it
// publishes map->camera_init CANDIDATES on /erc/aruco_camera_init_candidate.
// Multiple known markers constrain planar x/y/yaw. After global initialization,
// one known marker may constrain x/y only while the current GICP/FAST-LIO yaw is
// preserved. This is deliberately
// useful precisely when GICP itself is degenerate (flat/featureless terrain
// starves point-to-plane matching of a constraint in some direction) --
// ArUco's failure modes are unrelated to LiDAR geometry, so it can correct
// camera_init in situations GICP alone cannot. Both sources are fused through
// the SAME jump-gate + low-pass logic (applyCorrection), so map->camera_init
// still has a single owner/publisher (CLAUDE.md §6.2) -- this node just now
// accepts two independent inputs for it instead of one.
//
// TF authority (no double publishing, CLAUDE.md §6.2):
//   map -> camera_init : THIS node (GICP + ArUco candidate, once frozen)
//   camera_init -> body: FAST-LIO
//   map -> datum       : aruco_map_anchor.py (SE(2) fit, frozen after convergence)
//
// GICP convention: target ≈ T * source, with source in camera_init and target in
// map, so T == pose of camera_init in map == the map->camera_init transform we
// publish. The initial guess for align() is the current estimate of that same T.

#include <algorithm>
#include <cmath>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>

using PointT = pcl::PointXYZI;
using Cloud = pcl::PointCloud<PointT>;

class MapAnchor : public rclcpp::Node
{
public:
  MapAnchor() : rclcpp::Node("map_anchor")
  {
    // ---- parameters ----
    prior_map_path_ = declare_parameter<std::string>("prior_map_path", "");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "camera_init");
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/cloud_registered");
    voxel_leaf_ = declare_parameter<double>("voxel_leaf", 0.3);
    accum_window_sec_ = declare_parameter<double>("accum_window_sec", 1.0);
    gicp_max_corr_dist_ = declare_parameter<double>("gicp_max_corr_dist", 1.0);
    gicp_max_iter_ = declare_parameter<int>("gicp_max_iter", 30);
    update_period_ = declare_parameter<double>("update_period", 1.0);
    fitness_max_ = declare_parameter<double>("fitness_max", 0.3);
    max_trans_step_ = declare_parameter<double>("max_trans_step", 1.0);
    max_rot_step_ = declare_parameter<double>("max_rot_step", 0.5);
    lowpass_alpha_ = declare_parameter<double>("lowpass_alpha", 0.3);
    planar_correction_ = declare_parameter<bool>("planar_correction", true);
    auto init_xyz = declare_parameter<std::vector<double>>("init_xyz", {0.0, 0.0, 0.0});
    double init_yaw = declare_parameter<double>("init_yaw", 0.0);
    double tf_rate = declare_parameter<double>("tf_publish_rate", 20.0);

    // Second correction source: ArUco camera_init candidates (see file header).
    aruco_candidate_topic_ =
      declare_parameter<std::string>("aruco_candidate_topic", "/erc/aruco_camera_init_candidate");
    aruco_lowpass_alpha_ = declare_parameter<double>("aruco_lowpass_alpha", 0.3);
    aruco_residual_max_ = declare_parameter<double>("aruco_residual_max", 0.5);
    wait_for_aruco_initialization_ =
      declare_parameter<bool>("wait_for_aruco_initialization", false);
    aruco_init_min_candidates_ = declare_parameter<int>("aruco_init_min_candidates", 3);
    aruco_init_consistency_trans_ =
      declare_parameter<double>("aruco_init_consistency_trans", 0.5);
    aruco_init_consistency_rot_ =
      declare_parameter<double>("aruco_init_consistency_rot", 0.25);
    initialized_ = !wait_for_aruco_initialization_;

    // ---- initial map->camera_init estimate (start pose ~ known, ERC §3.4) ----
    current_T_ = Eigen::Matrix4f::Identity();
    current_T_.block<3, 3>(0, 0) =
      Eigen::AngleAxisf(static_cast<float>(init_yaw), Eigen::Vector3f::UnitZ()).toRotationMatrix();
    if (init_xyz.size() == 3) {
      current_T_(0, 3) = static_cast<float>(init_xyz[0]);
      current_T_(1, 3) = static_cast<float>(init_xyz[1]);
      current_T_(2, 3) = static_cast<float>(init_xyz[2]);
    }

    // ---- load + downsample the prior map (GICP target) ----
    if (!loadPriorMap()) {
      if (wait_for_aruco_initialization_) {
        RCLCPP_WARN(
          get_logger(),
          "prior map load failed; running ArUco-only global initialization (GICP disabled)");
      } else {
        RCLCPP_FATAL(get_logger(), "prior map load failed; map_anchor will only publish the "
                                   "initial (uncorrected) TF.");
      }
    }

    gicp_.setMaxCorrespondenceDistance(gicp_max_corr_dist_);
    gicp_.setMaximumIterations(gicp_max_iter_);
    gicp_.setTransformationEpsilon(1e-6);
    gicp_.setEuclideanFitnessEpsilon(1e-4);
    if (target_ && !target_->empty()) {
      gicp_.setInputTarget(target_);
    }

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    fitness_pub_ = create_publisher<std_msgs::msg::Float32>("/erc/localization_fitness", 10);
    // Latched state: diagnostics/Nav2 started later must still learn whether the
    // global pose has been initialized.
    initialized_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/erc/localization_initialized", rclcpp::QoS(1).reliable().transient_local());

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&MapAnchor::cloudCb, this, std::placeholders::_1));
    aruco_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      aruco_candidate_topic_, 10,
      std::bind(&MapAnchor::arucoCandidateCb, this, std::placeholders::_1));

    // Steady TF broadcast, independent of the (slower) GICP updates.
    tf_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, tf_rate)),
      std::bind(&MapAnchor::broadcastTf, this));
    gicp_timer_ = create_wall_timer(
      std::chrono::duration<double>(update_period_),
      std::bind(&MapAnchor::runGicp, this));

    RCLCPP_INFO(get_logger(),
                "map_anchor up: %s -> %s, prior='%s' (%zu pts), source topic '%s', "
                "aruco candidate topic '%s'",
                map_frame_.c_str(), odom_frame_.c_str(), prior_map_path_.c_str(),
                target_ ? target_->size() : 0, cloud_topic_.c_str(),
                aruco_candidate_topic_.c_str());
    publishInitializationState();
  }

private:
  bool loadPriorMap()
  {
    if (prior_map_path_.empty()) {
      RCLCPP_ERROR(get_logger(), "prior_map_path is empty");
      return false;
    }
    auto raw = std::make_shared<Cloud>();
    if (pcl::io::loadPCDFile<PointT>(prior_map_path_, *raw) < 0) {
      RCLCPP_ERROR(get_logger(), "could not read %s", prior_map_path_.c_str());
      return false;
    }
    target_ = downsample(raw);
    return target_ && !target_->empty();
  }

  Cloud::Ptr downsample(const Cloud::Ptr & in)
  {
    if (voxel_leaf_ <= 0.0) return in;
    auto out = std::make_shared<Cloud>();
    pcl::VoxelGrid<PointT> vg;
    vg.setLeafSize(voxel_leaf_, voxel_leaf_, voxel_leaf_);
    vg.setInputCloud(in);
    vg.filter(*out);
    return out;
  }

  void cloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    auto cloud = std::make_shared<Cloud>();
    pcl::fromROSMsg(*msg, *cloud);
    if (cloud->empty()) return;
    // Single-threaded executor serializes cloudCb/runGicp -> no lock needed.
    buffer_.emplace_back(now(), cloud);
    // prune anything older than the accumulation window
    const rclcpp::Duration window = rclcpp::Duration::from_seconds(accum_window_sec_);
    while (!buffer_.empty() && (now() - buffer_.front().first) > window) {
      buffer_.pop_front();
    }
  }

  // Merge the rolling window of live scans (camera_init frame) into one downsampled source.
  Cloud::Ptr buildSource()
  {
    auto merged = std::make_shared<Cloud>();
    for (const auto & entry : buffer_) {
      *merged += *entry.second;
    }
    if (merged->empty()) return merged;
    return downsample(merged);
  }

  void runGicp()
  {
    if (!initialized_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "GICP is waiting for a consistent multi-marker ArUco global initialization");
      return;
    }
    if (!target_ || target_->empty()) return;
    auto source = buildSource();
    if (!source || source->size() < 100) return;  // not enough geometry yet

    gicp_.setInputSource(source);
    Cloud aligned;
    gicp_.align(aligned, current_T_);

    std_msgs::msg::Float32 fmsg;
    fmsg.data = static_cast<float>(gicp_.getFitnessScore());
    fitness_pub_->publish(fmsg);

    if (!gicp_.hasConverged()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "GICP did not converge");
      return;
    }
    if (fmsg.data > fitness_max_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                           "GICP fitness %.3f > %.3f (rejected)", fmsg.data, fitness_max_);
      return;
    }

    Eigen::Matrix4f correction = gicp_.getFinalTransformation();
    if (planar_correction_) {
      correction = projectToGravityAlignedPlanar(correction);
    }
    applyCorrection(correction, lowpass_alpha_, "GICP");
  }

  // Both FAST-LIO camera_init frames are gravity aligned. A free 6-DoF GICP fit can
  // nevertheless tilt one room/map onto another local minimum (observed as ~10 deg
  // roll/pitch with a deceptively good fitness), which is unsafe for a planar Nav2
  // costmap. Keep xyz translation but project rotation to yaw when configured.
  Eigen::Matrix4f projectToGravityAlignedPlanar(const Eigen::Matrix4f & transform) const
  {
    const Eigen::Matrix3f rotation = transform.block<3, 3>(0, 0);
    const float yaw = std::atan2(rotation(1, 0), rotation(0, 0));
    Eigen::Matrix4f planar = Eigen::Matrix4f::Identity();
    planar.block<3, 3>(0, 0) =
      Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    planar.block<3, 1>(0, 3) = transform.block<3, 1>(0, 3);
    return planar;
  }

  // Candidate map->camera_init from aruco_map_anchor.py. Multi-marker candidates
  // constrain planar x/y/yaw. A one-marker candidate sets a large covariance[35]
  // sentinel and constrains x/y only after initialization. z/pitch/roll always remain
  // exactly as GICP/FAST-LIO currently have them.
  void arucoCandidateCb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    // covariance[0]/[7] carry the fit's RMS residual [m] as a quality scalar (see
    // aruco_map_anchor.py's _update_camera_init_candidate) -- not a real covariance.
    const double residual = msg->pose.covariance[0];
    constexpr double kTranslationOnlyYawVariance = 1.0e5;
    const bool translation_only =
      msg->pose.covariance[35] >= kTranslationOnlyYawVariance;
    if (!std::isfinite(residual) || residual > aruco_residual_max_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                           "ArUco camera_init candidate residual %.3f > %.3f (rejected)",
                           residual, aruco_residual_max_);
      return;
    }
    // One landmark cannot initialize global yaw. Even if a malformed/reordered launch
    // delivers such a candidate early, never count it toward the multi-marker global
    // initializer.
    if (translation_only && !initialized_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "translation-only ArUco candidate received before global initialization; rejected");
      return;
    }
    const double qx = msg->pose.pose.orientation.x, qy = msg->pose.pose.orientation.y;
    const double qz = msg->pose.pose.orientation.z, qw = msg->pose.pose.orientation.w;
    const float new_yaw = static_cast<float>(
      std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)));

    Eigen::Matrix4f T_new = current_T_;
    if (!translation_only) {
      // Preserve current pitch/roll while applying multi-marker yaw.
      const Eigen::Vector3f euler = current_T_.block<3, 3>(0, 0).eulerAngles(2, 1, 0);
      const float pitch = euler[1];
      const float roll = euler[2];
      T_new.block<3, 3>(0, 0) =
        (Eigen::AngleAxisf(new_yaw, Eigen::Vector3f::UnitZ()) *
         Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
         Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())).toRotationMatrix();
    }
    T_new(0, 3) = static_cast<float>(msg->pose.pose.position.x);
    T_new(1, 3) = static_cast<float>(msg->pose.pose.position.y);
    T_new(2, 3) = current_T_(2, 3);  // keep current height

    if (!initialized_) {
      collectInitializationCandidate(T_new, residual);
      return;
    }
    const bool applied = applyCorrection(
      T_new, aruco_lowpass_alpha_,
      translation_only ? "ArUco-1tag-xy" : "ArUco-multitag");
    if (translation_only && applied) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "accepted one-marker ArUco x/y candidate; current yaw preserved");
    }
  }

  // Arbitrary-start localization needs a one-time correction that may legitimately be
  // much larger than the normal jump gate. Require several mutually-consistent
  // multi-marker candidates, then apply the absolute pose once before enabling GICP.
  void collectInitializationCandidate(const Eigen::Matrix4f & candidate, double residual)
  {
    if (init_candidate_count_ == 0) {
      last_init_candidate_ = candidate;
      init_candidate_count_ = 1;
    } else {
      const Eigen::Vector3f t_last = last_init_candidate_.block<3, 1>(0, 3);
      const Eigen::Vector3f t_new = candidate.block<3, 1>(0, 3);
      const Eigen::Quaternionf q_last(
        Eigen::Matrix3f(last_init_candidate_.block<3, 3>(0, 0)));
      const Eigen::Quaternionf q_new(Eigen::Matrix3f(candidate.block<3, 3>(0, 0)));
      const float dt = (t_new - t_last).norm();
      const float dang = q_last.angularDistance(q_new);
      if (dt > aruco_init_consistency_trans_ || dang > aruco_init_consistency_rot_) {
        RCLCPP_WARN(
          get_logger(),
          "ArUco initializer reset: candidates disagree (dt=%.2f m, dang=%.2f rad)",
          dt, dang);
        init_candidate_count_ = 1;
      } else {
        ++init_candidate_count_;
      }
      last_init_candidate_ = candidate;
    }

    RCLCPP_INFO(
      get_logger(), "ArUco global initializer candidate %d/%d (residual=%.3f m)",
      init_candidate_count_, std::max(1, aruco_init_min_candidates_), residual);
    if (init_candidate_count_ < std::max(1, aruco_init_min_candidates_)) {
      return;
    }

    current_T_ = last_init_candidate_;
    initialized_ = true;
    publishInitializationState();
    RCLCPP_INFO(
      get_logger(),
      "ArUco global initialization accepted; enabling prior-map GICP refinement");
  }

  void publishInitializationState()
  {
    std_msgs::msg::Bool msg;
    msg.data = initialized_;
    initialized_pub_->publish(msg);
  }

  // Shared by both correction sources (GICP, ArUco candidate): reject discontinuous
  // jumps (§6.4 safety valve), then low-pass so the pose never snaps.
  bool applyCorrection(const Eigen::Matrix4f & T_new, double alpha, const char * source)
  {
    const Eigen::Vector3f t_cur = current_T_.block<3, 1>(0, 3);
    const Eigen::Vector3f t_new = T_new.block<3, 1>(0, 3);
    const Eigen::Quaternionf q_cur(Eigen::Matrix3f(current_T_.block<3, 3>(0, 0)));
    const Eigen::Quaternionf q_new(Eigen::Matrix3f(T_new.block<3, 3>(0, 0)));
    const float dt = (t_new - t_cur).norm();
    const float dang = q_cur.angularDistance(q_new);
    if (dt > max_trans_step_ || dang > max_rot_step_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                           "[%s] correction jump too large (dt=%.2f m, dang=%.2f rad) — rejected",
                           source, dt, dang);
      return false;
    }

    const float a = static_cast<float>(alpha);
    const Eigen::Vector3f t = (1.0f - a) * t_cur + a * t_new;
    Eigen::Quaternionf q = q_cur.slerp(a, q_new);
    q.normalize();
    current_T_.setIdentity();
    current_T_.block<3, 3>(0, 0) = q.toRotationMatrix();
    current_T_.block<3, 1>(0, 3) = t;
    return true;
  }

  void broadcastTf()
  {
    const Eigen::Vector3f t = current_T_.block<3, 1>(0, 3);
    Eigen::Quaternionf q(Eigen::Matrix3f(current_T_.block<3, 3>(0, 0)));
    q.normalize();

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = map_frame_;     // parent
    tf.child_frame_id = odom_frame_;     // child (== FAST-LIO camera_init)
    tf.transform.translation.x = t.x();
    tf.transform.translation.y = t.y();
    tf.transform.translation.z = t.z();
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf);
  }

  // params
  std::string prior_map_path_, map_frame_, odom_frame_, cloud_topic_, aruco_candidate_topic_;
  double voxel_leaf_, accum_window_sec_, gicp_max_corr_dist_, update_period_;
  double fitness_max_, max_trans_step_, max_rot_step_, lowpass_alpha_;
  double aruco_lowpass_alpha_, aruco_residual_max_;
  double aruco_init_consistency_trans_, aruco_init_consistency_rot_;
  int gicp_max_iter_;
  int aruco_init_min_candidates_;
  bool wait_for_aruco_initialization_, initialized_, planar_correction_;
  int init_candidate_count_{0};
  Eigen::Matrix4f last_init_candidate_{Eigen::Matrix4f::Identity()};

  // state
  Eigen::Matrix4f current_T_;               // map -> camera_init (current estimate)
  Cloud::Ptr target_;                       // downsampled prior map (map frame)
  std::deque<std::pair<rclcpp::Time, Cloud::Ptr>> buffer_;  // rolling live scans
  pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp_;

  // ros
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr fitness_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr initialized_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr aruco_sub_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  rclcpp::TimerBase::SharedPtr gicp_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapAnchor>());
  rclcpp::shutdown();
  return 0;
}
