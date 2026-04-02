// publish_realsense.cpp
// RealSense D435i から RGB(BGR) 画像・カラーにアラインした Depth 画像・
// カメラ内部パラメータを publish する ROS2 C++ ノード

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

#include <librealsense2/rs.hpp>

#include <chrono>
#include <sstream>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class RealSensePublisherNode : public rclcpp::Node
{
public:
  RealSensePublisherNode()
  : rclcpp::Node("realsense_publisher")
  {
    // パラメータ宣言（Python 実装と同等）
    color_width_  = this->declare_parameter<int>("color_width", 640);
    color_height_ = this->declare_parameter<int>("color_height", 480);
    depth_width_  = this->declare_parameter<int>("depth_width", 640);
    depth_height_ = this->declare_parameter<int>("depth_height", 480);
    fps_          = this->declare_parameter<int>("fps", 30);
    color_topic_  = this->declare_parameter<std::string>("color_topic", "camera/color/image_raw");
    depth_topic_  = this->declare_parameter<std::string>("depth_topic", "camera/depth/image_raw");

    // QoS: RELIABLE / KEEP_LAST(5)
    rclcpp::QoS qos(rclcpp::KeepLast(5));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

    pub_color_ = this->create_publisher<sensor_msgs::msg::Image>(color_topic_, qos);
    pub_depth_ = this->create_publisher<sensor_msgs::msg::Image>(depth_topic_, qos);
    pub_realsense_info_ = this->create_publisher<std_msgs::msg::String>("realsense_info", qos);

    // RealSense パイプライン設定
    try {
      // ストリーム設定（Python 実装を踏襲）
      config_.enable_stream(RS2_STREAM_DEPTH,
                            depth_width_,
                            depth_height_,
                            RS2_FORMAT_Z16,
                            fps_);
      config_.enable_stream(RS2_STREAM_COLOR,
                            color_width_,
                            color_height_,
                            RS2_FORMAT_BGR8,
                            fps_);

      // パイプライン開始
      profile_ = pipeline_.start(config_);

      // 深度スケール取得
      auto depth_sensor = profile_.get_device().first<rs2::depth_sensor>();
      depth_scale_ = depth_sensor.get_depth_scale();

      // カラーストリームの内部パラメータ取得
      auto color_stream = profile_.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
      auto intr = color_stream.get_intrinsics();
      fx_ = intr.fx;
      fy_ = intr.fy;

      // Depth を Color に合わせてアライン
      align_to_color_ = std::make_unique<rs2::align>(RS2_STREAM_COLOR);

      RCLCPP_INFO(this->get_logger(), "RealSense パイプラインを開始しました。");
    } catch (const rs2::error & e) {
      RCLCPP_ERROR(this->get_logger(),
                   "RealSense の起動に失敗しました (rs2::error): %s", e.what());
      throw;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(),
                   "RealSense の起動に失敗しました (std::exception): %s", e.what());
      throw;
    }

    // タイマーでフレーム取得 & publish
    auto period = std::chrono::duration<double>(1.0 / std::max(1, fps_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&RealSensePublisherNode::timerCallback, this));
  }

  ~RealSensePublisherNode() override
  {
    try {
      pipeline_.stop();
    } catch (...) {
      // 例外は握りつぶす
    }
  }

private:
  void timerCallback()
  {
    rs2::frameset frames;
    try {
      frames = pipeline_.wait_for_frames(1000);  // 1 秒タイムアウト
    } catch (const rs2::error & e) {
      RCLCPP_WARN(this->get_logger(), "フレーム取得失敗 (rs2::error): %s", e.what());
      return;
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "フレーム取得失敗 (std::exception): %s", e.what());
      return;
    }

    // Depth を Color にアライン
    rs2::frameset aligned_frames = frames;
    if (align_to_color_) {
      aligned_frames = align_to_color_->process(frames);
    }

    rs2::video_frame color_frame = aligned_frames.get_color_frame();
    rs2::depth_frame depth_frame = aligned_frames.get_depth_frame();

    if (!color_frame || !depth_frame) {
      return;
    }

    const auto color_width  = color_frame.get_width();
    const auto color_height = color_frame.get_height();
    const auto depth_width  = depth_frame.get_width();
    const auto depth_height = depth_frame.get_height();

    // BGR カラー画像メッセージを構築
    sensor_msgs::msg::Image color_msg;
    color_msg.header.stamp = this->get_clock()->now();
    color_msg.header.frame_id = "camera_color_optical_frame";
    color_msg.height = static_cast<uint32_t>(color_height);
    color_msg.width  = static_cast<uint32_t>(color_width);
    color_msg.encoding = "bgr8";
    color_msg.is_bigendian = 0;
    color_msg.step = static_cast<uint32_t>(color_width * 3);
    color_msg.data.resize(static_cast<size_t>(color_width * color_height * 3));

    std::memcpy(color_msg.data.data(),
                color_frame.get_data(),
                color_msg.data.size());

    pub_color_->publish(color_msg);

    // Depth 画像メッセージを構築 (16UC1)
    sensor_msgs::msg::Image depth_msg;
    depth_msg.header.stamp = color_msg.header.stamp;  // 同期
    depth_msg.header.frame_id = "camera_color_optical_frame";
    depth_msg.height = static_cast<uint32_t>(depth_height);
    depth_msg.width  = static_cast<uint32_t>(depth_width);
    depth_msg.encoding = "16UC1";
    depth_msg.is_bigendian = 0;
    depth_msg.step = static_cast<uint32_t>(depth_width * 2);  // 16bit = 2 bytes
    depth_msg.data.resize(static_cast<size_t>(depth_width * depth_height * 2));

    std::memcpy(depth_msg.data.data(),
                depth_frame.get_data(),
                depth_msg.data.size());

    pub_depth_->publish(depth_msg);

    // realsense_info (JSON 文字列) を publish
    std_msgs::msg::String info_msg;
    std::ostringstream oss;
    oss << "{"
        << "\"depth_scale\":" << depth_scale_ << ","
        << "\"fx\":" << fx_ << ","
        << "\"fy\":" << fy_
        << "}";
    info_msg.data = oss.str();
    pub_realsense_info_->publish(info_msg);
  }

private:
  // ROS2
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_color_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_realsense_info_;
  rclcpp::TimerBase::SharedPtr timer_;

  // RealSense
  rs2::pipeline pipeline_;
  rs2::config config_;
  rs2::pipeline_profile profile_;
  std::unique_ptr<rs2::align> align_to_color_;

  // パラメータ・内部状態
  int color_width_;
  int color_height_;
  int depth_width_;
  int depth_height_;
  int fps_;
  std::string color_topic_;
  std::string depth_topic_;

  float depth_scale_{0.0f};
  float fx_{0.0f};
  float fy_{0.0f};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RealSensePublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

