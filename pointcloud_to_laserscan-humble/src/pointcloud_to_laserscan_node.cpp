/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

/*
 * Author: Paul Bovbel
 */

#include "pointcloud_to_laserscan/pointcloud_to_laserscan_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2_ros/create_timer_ros.h"

namespace pointcloud_to_laserscan
{

PointCloudToLaserScanNode::PointCloudToLaserScanNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("pointcloud_to_laserscan", options)
{
  target_frame_ = this->declare_parameter("target_frame", "");
  tolerance_ = this->declare_parameter("transform_tolerance", 0.01);
  // TODO(hidmic): adjust default input queue size based on actual concurrency levels
  // achievable by the associated executor
  input_queue_size_ = this->declare_parameter(
    "queue_size", static_cast<int>(std::thread::hardware_concurrency()));
  min_height_ = this->declare_parameter("min_height", std::numeric_limits<double>::min());
  max_height_ = this->declare_parameter("max_height", std::numeric_limits<double>::max());
  angle_min_ = this->declare_parameter("angle_min", -M_PI);
  angle_max_ = this->declare_parameter("angle_max", M_PI);
  angle_increment_ = this->declare_parameter("angle_increment", M_PI / 180.0);
  scan_time_ = this->declare_parameter("scan_time", 1.0 / 30.0);
  range_min_ = this->declare_parameter("range_min", 0.0);
  range_max_ = this->declare_parameter("range_max", std::numeric_limits<double>::max());
  inf_epsilon_ = this->declare_parameter("inf_epsilon", 1.0);
  use_inf_ = this->declare_parameter("use_inf", true);
  apply_slope_filter_ = this->declare_parameter("apply_slope_filter", true);
  max_slope_angle_ = this->declare_parameter("max_slope_angle", 15.0) * M_PI / 180.0;
  normal_k_search_ = this->declare_parameter("normal_k_search", 20);
  voxel_leaf_size_ = this->declare_parameter("voxel_leaf_size", 0.11);
  //search_radius_ = this->declare_parameter("search_radius", 0.15); //ksearchの代わり

  pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
  filtered_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("angle_filtered_pointcloud", 10);
  corrected_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("corrected_pointcloud", 10);

  using std::placeholders::_1;
  // if pointcloud target frame specified, we need to filter by transform availability
  if (!target_frame_.empty()) {
    tf2_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_);
    message_filter_ = std::make_unique<MessageFilter>(
      sub_, *tf2_, target_frame_, input_queue_size_,
      this->get_node_logging_interface(),
      this->get_node_clock_interface());
    message_filter_->registerCallback(
      std::bind(&PointCloudToLaserScanNode::cloudCallback, this, _1));
  } else {  // otherwise setup direct subscription
    sub_.registerCallback(std::bind(&PointCloudToLaserScanNode::cloudCallback, this, _1));
  }

  subscription_listener_thread_ = std::thread(
    std::bind(&PointCloudToLaserScanNode::subscriptionListenerThreadLoop, this));
}

PointCloudToLaserScanNode::~PointCloudToLaserScanNode()
{
  alive_.store(false);
  subscription_listener_thread_.join();
}

void PointCloudToLaserScanNode::subscriptionListenerThreadLoop()
{
  rclcpp::Context::SharedPtr context = this->get_node_base_interface()->get_context();

  const std::chrono::milliseconds timeout(100);
  while (rclcpp::ok(context) && alive_.load()) {
    int subscription_count =
      pub_->get_subscription_count() +
      pub_->get_intra_process_subscription_count() +
      filtered_cloud_pub_->get_subscription_count() +
      filtered_cloud_pub_->get_intra_process_subscription_count() +
      corrected_cloud_pub_->get_subscription_count() +
      corrected_cloud_pub_->get_intra_process_subscription_count();
    if (subscription_count > 0) {
      if (!sub_.getSubscriber()) {
        RCLCPP_INFO(
          this->get_logger(),
          "Got a subscriber to filtered outputs, starting pointcloud subscriber");
        rclcpp::SensorDataQoS qos;
        qos.keep_last(input_queue_size_);
        sub_.subscribe(this, "cloud_in", qos.get_rmw_qos_profile());
      }
    } else if (sub_.getSubscriber()) {
      RCLCPP_INFO(
        this->get_logger(),
        "No subscribers to filtered outputs, shutting down pointcloud subscriber");
      sub_.unsubscribe();
    }
    rclcpp::Event::SharedPtr event = this->get_graph_event();
    this->wait_for_graph_change(event, timeout);
  }
  sub_.unsubscribe();
}

void PointCloudToLaserScanNode::cloudCallback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
{
  // build laserscan output
  auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
  scan_msg->header = cloud_msg->header;
  if (!target_frame_.empty()) {
    scan_msg->header.frame_id = target_frame_;
  }

  scan_msg->angle_min = angle_min_;
  scan_msg->angle_max = angle_max_;
  scan_msg->angle_increment = angle_increment_;
  scan_msg->time_increment = 0.0;
  scan_msg->scan_time = scan_time_;
  scan_msg->range_min = range_min_;
  scan_msg->range_max = range_max_;

  // determine amount of rays to create
  uint32_t ranges_size = std::ceil(
    (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);

  // determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
  if (use_inf_) {
    scan_msg->ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
  } else {
    scan_msg->ranges.assign(ranges_size, scan_msg->range_max + inf_epsilon_);
  }

  // Transform cloud if necessary
  if (scan_msg->header.frame_id != cloud_msg->header.frame_id) {
    try {
      auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
      tf2_->transform(*cloud_msg, *cloud, target_frame_, tf2::durationFromSec(tolerance_));
      cloud_msg = cloud;
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Transform failure: " << ex.what());
      return;
    }
  }

  corrected_cloud_pub_->publish(*cloud_msg);

  // Convert cloud to PCL and optionally apply the slope filter before creating LaserScan.
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::fromROSMsg(*cloud_msg, pcl_cloud);

  pcl::PointCloud<pcl::PointXYZ> filtered_pcl;
  filtered_pcl.is_dense = false;
  filtered_pcl.width = 0;
  filtered_pcl.height = 1;

  if (apply_slope_filter_) {
    pcl::PointCloud<pcl::PointXYZ> downsampled_cloud;
    if (voxel_leaf_size_ > 0.0 && pcl_cloud.points.size() > 0) {
      pcl::VoxelGrid<pcl::PointXYZ> vg;
      vg.setInputCloud(pcl_cloud.makeShared());
      vg.setLeafSize(
        static_cast<float>(voxel_leaf_size_),
        static_cast<float>(voxel_leaf_size_),
        static_cast<float>(voxel_leaf_size_));
      vg.filter(downsampled_cloud);
      downsampled_cloud.header = pcl_cloud.header;
    } else {
      downsampled_cloud = pcl_cloud;
    }

    pcl::PointCloud<pcl::Normal> normals;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(downsampled_cloud.makeShared());
    ne.setSearchMethod(tree);
    //ne.setRadiusSearch(search_radius_); // normal_k_search_の代わり
    ne.setKSearch(normal_k_search_);
    ne.compute(normals);

    filtered_pcl.header = downsampled_cloud.header;
    for (size_t i = 0; i < downsampled_cloud.points.size() && i < normals.points.size(); ++i) {
      const auto & pt = downsampled_cloud.points[i];
      const auto & normal = normals.points[i];
      if (!pcl::isFinite(pt) || !pcl::isFinite(normal)) {
        continue;
      }

      if (pt.z > max_height_ || pt.z < min_height_) {
        continue;
      }

      double nz = normal.normal_z;
      double slope_angle = std::acos(std::clamp(std::fabs(nz), 0.0, 1.0));
      if (slope_angle <= max_slope_angle_) {
        continue;
      }

      filtered_pcl.points.push_back(pt);
    }
  } else {
    filtered_pcl = pcl_cloud;
  }

  filtered_pcl.width = filtered_pcl.points.size();
  filtered_pcl.height = filtered_pcl.width > 0 ? 1 : 0;

  sensor_msgs::msg::PointCloud2 filtered_cloud_msg;
  pcl::toROSMsg(filtered_pcl, filtered_cloud_msg);
  filtered_cloud_msg.header = cloud_msg->header;
  filtered_cloud_pub_->publish(filtered_cloud_msg);

  // Iterate filtered points to build laserscan output
  for (const auto & pt : filtered_pcl.points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
      continue;
    }

    double range = hypot(pt.x, pt.y);
    if (range < range_min_) {
      continue;
    }
    if (range > range_max_) {
      continue;
    }

    double angle = atan2(pt.y, pt.x);
    if (angle < scan_msg->angle_min || angle > scan_msg->angle_max) {
      continue;
    }

    int index = (angle - scan_msg->angle_min) / scan_msg->angle_increment;
    if (index < 0 || static_cast<size_t>(index) >= scan_msg->ranges.size()) {
      continue;
    }
    if (range < scan_msg->ranges[index]) {
      scan_msg->ranges[index] = range;
    }
  }

  pub_->publish(std::move(scan_msg));
}

}  // namespace pointcloud_to_laserscan

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_to_laserscan::PointCloudToLaserScanNode)
