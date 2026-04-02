#ifndef RVIZ_NAV2_ARUCO_ID_DISPLAY_HPP
#define RVIZ_NAV2_ARUCO_ID_DISPLAY_HPP

#include <std_msgs/msg/float32.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <QLabel>
#include <QVBoxLayout>
#include <QTimer>
#include <QMetaObject>
#include <memory>
#ifndef Q_MOC_RUN
    #include <rviz_common/panel.hpp>
#endif

namespace rviz_nav2 {
    class ArucoIdDisplay : public rviz_common::Panel {
        Q_OBJECT
      public:
        ArucoIdDisplay(QWidget *parent = nullptr);
        virtual ~ArucoIdDisplay();

        virtual void onInitialize() override;
        virtual void load(const rviz_common::Config &config) override;
        virtual void save(rviz_common::Config config) const override;

      private Q_SLOTS:
        void updateDisplay();

      private:
        void processMessage(std_msgs::msg::Float32::ConstSharedPtr msg);
        bool isMessageDetected() const;

        // ROS2 node and subscription
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
        std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;

        // UI elements
        QLabel *status_label_;
        QLabel *id_label_;
        QTimer *qt_timer_;

        // Message detection tracking
        rclcpp::Time last_message_time_;
        rclcpp::Clock::SharedPtr clock_;
        double timeout_seconds_;
        float current_value_;
    };
} // namespace rviz_nav2

#endif // RVIZ_NAV2_ARUCO_ID_DISPLAY_HPP

