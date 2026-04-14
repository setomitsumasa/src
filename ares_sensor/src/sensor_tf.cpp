//
// sensor_tf: map -> odom, odom -> base_link, base_link -> imu_link, base_link -> gps_link,
//            base_link -> camera_link, livox_frame, camera_color_frame, camera_color_optical_frame,
//            camera_depth_frame, camera_depth_optical_frame を publish するノード
// ROS2 Humble

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <cmath>
#include <mutex>

/**
 * map -> odom, odom -> base_link, base_link -> imu_link の TF を定期的に publish するノード。
 */
class SensorTfNode : public rclcpp::Node
{
public:
    SensorTfNode()
    : rclcpp::Node("sensor_tf_node")
    {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
        imu_topic_ = this->declare_parameter<std::string>("imu_topic", "/imu/data");
        livox_stabilized_frame_id_ =
            this->declare_parameter<std::string>("livox_stabilized_frame_id", "livox_stabilized");
        enable_livox_stabilized_frame_ =
            this->declare_parameter<bool>("enable_livox_stabilized_frame", true);

        const int publish_rate = this->declare_parameter<int>("publish_rate", 50);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / publish_rate),
            std::bind(&SensorTfNode::publishTransforms, this));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_,
            rclcpp::QoS(50),
            std::bind(&SensorTfNode::imuCallback, this, std::placeholders::_1));

        // base_link -> camera_link のオフセット [m]
        camera_link_x_ = this->declare_parameter<double>("camera_link.x", 0.0);
        camera_link_y_ = this->declare_parameter<double>("camera_link.y", 0.0);
        camera_link_z_ = this->declare_parameter<double>("camera_link.z", 0.5);
        // base_link -> livox_frame のオフセット [m]（デフォルトは camera_link と同じ）
        livox_frame_x_ = this->declare_parameter<double>("livox_frame.x", camera_link_x_);
        livox_frame_y_ = this->declare_parameter<double>("livox_frame.y", camera_link_y_);
        livox_frame_z_ = this->declare_parameter<double>("livox_frame.z", camera_link_z_);
        // camera_link -> camera_color_frame のオフセット [m]（RealSense の色センサ位置）
        camera_color_offset_x_ = this->declare_parameter<double>("camera_color_offset.x", 0.0);
        camera_color_offset_y_ = this->declare_parameter<double>("camera_color_offset.y", 0.0);
        camera_color_offset_z_ = this->declare_parameter<double>("camera_color_offset.z", 0.0);
        // camera_link -> camera_depth_frame のオフセット [m]（地面から50cmになるよう z デフォルト 0.5）
        camera_depth_offset_x_ = this->declare_parameter<double>("camera_depth_offset.x", 0.0);
        camera_depth_offset_y_ = this->declare_parameter<double>("camera_depth_offset.y", 0.0);
        camera_depth_offset_z_ = this->declare_parameter<double>("camera_depth_offset.z", 0.0);
        // base_link -> gps_link のオフセット [m]
        gps_link_x_ = this->declare_parameter<double>("gps_link.x", 0.0);
        gps_link_y_ = this->declare_parameter<double>("gps_link.y", 0.0);
        gps_link_z_ = this->declare_parameter<double>("gps_link.z", 0.0);

        RCLCPP_INFO(this->get_logger(),
            "sensor_tf_node: publishing TF at %d Hz (imu_topic=%s, livox_stabilized_frame=%s)",
            publish_rate, imu_topic_.c_str(), livox_stabilized_frame_id_.c_str());
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        const auto & q = msg->orientation;
        if (!std::isfinite(q.x) || !std::isfinite(q.y) ||
            !std::isfinite(q.z) || !std::isfinite(q.w))
        {
            return;
        }

        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        if (quat.length2() < 1e-12) {
            return;
        }

        quat.normalize();

        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        std::lock_guard<std::mutex> lock(imu_mutex_);
        current_roll_ = roll;
        current_pitch_ = pitch;
        imu_received_ = true;
    }

    void publishTransforms()
    {
        const auto now = this->get_clock()->now();
        std::vector<geometry_msgs::msg::TransformStamped> transforms;

        transforms.push_back(createTransform(now, "map", "odom", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0));



        transforms.push_back(createTransform(now, "base_link", "imu_link", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0));
        transforms.push_back(createTransform(now, "base_link", "gps_link",
            gps_link_x_, gps_link_y_, gps_link_z_, 0.0, 0.0, 0.0, 1.0));

        // base_link -> camera_link: 緑軸(Y)+90deg のあと 青軸(Z)まわりに -90deg → Rz(-90°)*Ry(90°) = (0.5, 0.5, -0.5, 0.5)
        transforms.push_back(createTransform(now, "base_link", "camera_link",
            camera_link_x_, camera_link_y_, camera_link_z_,
            0.0, 0.0, 0.0, 1.0));
        // base_link -> livox_frame: デフォルトは camera_link と同じ位置・姿勢
        transforms.push_back(createTransform(now, "base_link", "livox_frame",
            livox_frame_x_, livox_frame_y_, livox_frame_z_,
            0.0, 0.0, 0.0, 1.0));
        if (enable_livox_stabilized_frame_) {
            double roll = 0.0;
            double pitch = 0.0;
            bool imu_received = false;
            {
                std::lock_guard<std::mutex> lock(imu_mutex_);
                roll = current_roll_;
                pitch = current_pitch_;
                imu_received = imu_received_;
            }

            if (!imu_received) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 5000,
                    "IMU orientation has not arrived yet. Publishing identity livox stabilization TF.");
            }

            tf2::Quaternion level_quat;
            level_quat.setRPY(-roll, -pitch, 0.0);
            level_quat.normalize();
            transforms.push_back(createTransform(
                now, "livox_frame", livox_stabilized_frame_id_,
                0.0, 0.0, 0.0,
                level_quat.x(), level_quat.y(), level_quat.z(), level_quat.w()));
        }
        // camera_color_frame: 回転なし（Z回転を入れると左右の移動がRvizで上下に見えるため）
        transforms.push_back(createTransform(now, "camera_link", "camera_color_frame",
            camera_color_offset_x_, camera_color_offset_y_, camera_color_offset_z_, 0.0, 0.0, 0.0, 1.0));
        // camera_depth_frame: RealSense座標系(Z前,X右,Y下) → ROS2座標系(X前,Y左,Z上) に変換
        // クォータニオン: (qx, qy, qz, qw) = (0.5, 0.5, -0.5, 0.5)
        transforms.push_back(createTransform(now, "camera_link", "camera_depth_frame",
            camera_depth_offset_x_, camera_depth_offset_y_, camera_depth_offset_z_, 0.0, 0.0, 0.0, 1.0));          
        // optical frame: Z が光軸（前）、X 右、Y 下。camera_*_frame から Rx(-90°) で変換
        // transforms.push_back(createTransform(now, "camera_color_frame", "camera_color_optical_frame",
        //     0.0, 0.0, 0.0, 0.5, 0.5, -0.5, 0.5));
        transforms.push_back(createTransform(now, "camera_color_frame", "camera_color_optical_frame",
            0.0, 0.0, 0.0, -0.5, 0.5, -0.5, 0.5));
        transforms.push_back(createTransform(now, "camera_depth_frame", "camera_depth_optical_frame",
            0.0, 0.0, 0.0, -0.5, 0.5, -0.5, 0.5));

        tf_broadcaster_->sendTransform(transforms);
    }

    geometry_msgs::msg::TransformStamped createTransform(
        const rclcpp::Time & stamp,
        const std::string & frame_id,
        const std::string & child_frame_id,
        double x, double y, double z,
        double qx, double qy, double qz, double qw)
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = stamp;
        t.header.frame_id = frame_id;
        t.child_frame_id = child_frame_id;
        t.transform.translation.x = x;
        t.transform.translation.y = y;
        t.transform.translation.z = z;
        t.transform.rotation.x = qx;
        t.transform.rotation.y = qy;
        t.transform.rotation.z = qz;
        t.transform.rotation.w = qw;
        return t;
    }

    // Z 軸まわりヨー角のみの変換（roll=0, pitch=0）
    geometry_msgs::msg::TransformStamped createTransformFromYaw(
        const rclcpp::Time & stamp,
        const std::string & frame_id,
        const std::string & child_frame_id,
        double x, double y, double z,
        double yaw_rad)
    {
        const double qz = std::sin(yaw_rad * 0.5);
        const double qw = std::cos(yaw_rad * 0.5);
        return createTransform(stamp, frame_id, child_frame_id, x, y, z, 0.0, 0.0, qz, qw);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::mutex imu_mutex_;

    double camera_link_x_ = 0.0, camera_link_y_ = 0.0, camera_link_z_ = 0.0;
    double livox_frame_x_ = 0.0, livox_frame_y_ = 0.0, livox_frame_z_ = 0.0;
    double camera_color_offset_x_ = 0.0, camera_color_offset_y_ = 0.0, camera_color_offset_z_ = 0.0;
    double camera_depth_offset_x_ = 0.0, camera_depth_offset_y_ = 0.0, camera_depth_offset_z_ = 0.0;
    double gps_link_x_ = 0.0, gps_link_y_ = 0.0, gps_link_z_ = 0.0;
    double current_roll_ = 0.0, current_pitch_ = 0.0;
    bool imu_received_ = false;
    std::string imu_topic_;
    std::string livox_stabilized_frame_id_;
    bool enable_livox_stabilized_frame_ = true;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorTfNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
