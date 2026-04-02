#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace ares_nav2
{

class RoverRecovery : public BT::StatefulActionNode
{
public:
  RoverRecovery(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("duration", 3.0, "seconds to back[sec]"),
      BT::InputPort<double>("back_speed", 0.3, "speed to back[rad/s]"),
      BT::InputPort<std::string>("cmd_vel_topic", "cmd_vel", "topic to publish velocity"),
    };
  }

  BT::NodeStatus onStart() override
  {
    if (!node_) {
      try {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
      } catch (const std::exception & e) {
        throw BT::RuntimeError("RoverRecovery requires [node] in blackboard: ", e.what());
      }
    }

    // 入力取得
    if (!getInput<double>("duration", duration_)) {
      duration_ = 3.0;
    }
    if (!getInput<double>("back_speed", speed_)) {
      speed_ = 0.3;
    }
    std::string topic = "cmd_vel";
    (void)getInput<std::string>("cmd_vel_topic", topic);

    // Publisher 準備
    if (!pub_ || std::string(pub_->get_topic_name()) != topic ) {
      pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(topic, rclcpp::SystemDefaultsQoS());
    }

    start_time_ = node_->now();
    publishZero();
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    const auto now = node_->now();
    const double elapsed = (now - start_time_).seconds();
    if (elapsed >= duration_) {
      publishZero();
      return BT::NodeStatus::SUCCESS;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = -speed_;
    pub_->publish(cmd);
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    publishZero();
  }

private:
  void publishZero() const {
    if (pub_) {
      const geometry_msgs::msg::Twist stop;
      pub_->publish(stop);
    }
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Time start_time_;
  double duration_{3.0};
  double speed_{0.3};
};

}  // namespace ares_nav2

// BehaviorTree.CPP のプラグイン登録
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<ares_nav2::RoverRecovery>("RoverRecovery");
}