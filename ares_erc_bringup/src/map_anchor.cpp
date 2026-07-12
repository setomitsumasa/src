// ERC Phase 1b — prior-map relocalization ("map anchor").
//
// Keeps the verified Ericsii FAST-LIO as the local odometry authority
// (it publishes camera_init -> body). This node adds the *global* correction:
// it GICP-aligns the live FAST-LIO cloud (/cloud_registered, expressed in the
// camera_init/odom frame) onto a saved prior map (.pcd, expressed in the map
// frame) and broadcasts a low-passed  map -> camera_init  transform.
//
// TF authority (no double publishing, CLAUDE.md §6.2):
//   map -> camera_init : THIS node (global correction)
//   camera_init -> body: FAST-LIO
//   map -> datum       : erc_waypoints.py (static yaw)
//
// GICP convention: target ≈ T * source, with source in camera_init and target in
// map, so T == pose of camera_init in map == the map->camera_init transform we
// publish. The initial guess for align() is the current estimate of that same T.

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
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
    auto init_xyz = declare_parameter<std::vector<double>>("init_xyz", {0.0, 0.0, 0.0});
    double init_yaw = declare_parameter<double>("init_yaw", 0.0);
    double tf_rate = declare_parameter<double>("tf_publish_rate", 20.0);

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
      RCLCPP_FATAL(get_logger(), "prior map load failed; map_anchor will only publish the "
                                 "initial (uncorrected) TF.");
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

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&MapAnchor::cloudCb, this, std::placeholders::_1));

    // Steady TF broadcast, independent of the (slower) GICP updates.
    tf_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, tf_rate)),
      std::bind(&MapAnchor::broadcastTf, this));
    gicp_timer_ = create_wall_timer(
      std::chrono::duration<double>(update_period_),
      std::bind(&MapAnchor::runGicp, this));

    RCLCPP_INFO(get_logger(),
                "map_anchor up: %s -> %s, prior='%s' (%zu pts), source topic '%s'",
                map_frame_.c_str(), odom_frame_.c_str(), prior_map_path_.c_str(),
                target_ ? target_->size() : 0, cloud_topic_.c_str());
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

    const Eigen::Matrix4f T_new = gicp_.getFinalTransformation();

    // Outlier gate: reject discontinuous jumps (bad match) — §6.4 safety valve.
    const Eigen::Vector3f t_cur = current_T_.block<3, 1>(0, 3);
    const Eigen::Vector3f t_new = T_new.block<3, 1>(0, 3);
    const Eigen::Quaternionf q_cur(Eigen::Matrix3f(current_T_.block<3, 3>(0, 0)));
    const Eigen::Quaternionf q_new(Eigen::Matrix3f(T_new.block<3, 3>(0, 0)));
    const float dt = (t_new - t_cur).norm();
    const float dang = q_cur.angularDistance(q_new);
    if (dt > max_trans_step_ || dang > max_rot_step_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                           "correction jump too large (dt=%.2f m, dang=%.2f rad) — rejected",
                           dt, dang);
      return;
    }

    // Low-pass the accepted correction so the pose never snaps discontinuously.
    const float a = static_cast<float>(lowpass_alpha_);
    const Eigen::Vector3f t = (1.0f - a) * t_cur + a * t_new;
    Eigen::Quaternionf q = q_cur.slerp(a, q_new);
    q.normalize();
    current_T_.setIdentity();
    current_T_.block<3, 3>(0, 0) = q.toRotationMatrix();
    current_T_.block<3, 1>(0, 3) = t;
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
  std::string prior_map_path_, map_frame_, odom_frame_, cloud_topic_;
  double voxel_leaf_, accum_window_sec_, gicp_max_corr_dist_, update_period_;
  double fitness_max_, max_trans_step_, max_rot_step_, lowpass_alpha_;
  int gicp_max_iter_;

  // state
  Eigen::Matrix4f current_T_;               // map -> camera_init (current estimate)
  Cloud::Ptr target_;                       // downsampled prior map (map frame)
  std::deque<std::pair<rclcpp::Time, Cloud::Ptr>> buffer_;  // rolling live scans
  pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp_;

  // ros
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr fitness_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
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
