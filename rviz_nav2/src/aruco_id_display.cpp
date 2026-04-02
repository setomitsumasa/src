#include <rviz_nav2/aruco_id_display.hpp>

#include <rviz_common/logging.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <sstream>
#include <iomanip>

namespace rviz_nav2 {
    ArucoIdDisplay::ArucoIdDisplay(QWidget *parent)
        : rviz_common::Panel(parent),
          status_label_(nullptr),
          id_label_(nullptr),
          qt_timer_(nullptr),
          last_message_time_(0, 0, RCL_ROS_TIME),
          clock_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)),
          timeout_seconds_(1.0),
          current_value_(0.0f) {
        
        // Initialize last_message_time_ with clock
        last_message_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());

        // Create UI layout
        QVBoxLayout *layout = new QVBoxLayout;
        
        // Status label
        status_label_ = new QLabel("None");
        status_label_->setStyleSheet("QLabel { font-size: 40px; font-weight: bold; padding: 30px; }");
        layout->addWidget(status_label_);

        // ID label (optional, shows the actual ID value)
        id_label_ = new QLabel("ID: --");
        id_label_->setStyleSheet("QLabel { font-size: 20px; padding: 10px; }");
        layout->addWidget(id_label_);

        setLayout(layout);
    }

    ArucoIdDisplay::~ArucoIdDisplay() {
        if (qt_timer_) {
            qt_timer_->stop();
        }
        if (executor_ && node_) {
            executor_->remove_node(node_);
        }
    }

    void ArucoIdDisplay::onInitialize() {
        // Create a node but don't add it to rviz2's executor
        // We'll use our own executor to process callbacks
        node_ = rclcpp::Node::make_shared("aruco_id_panel_node");
        executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(node_);

        // Create subscription
        subscription_ = node_->create_subscription<std_msgs::msg::Float32>(
            "/aruco/id",
            10,
            [this](const std_msgs::msg::Float32::SharedPtr msg) {
                this->processMessage(msg);
                // Update display immediately when message is received
                QMetaObject::invokeMethod(this, "updateDisplay", Qt::QueuedConnection);
            });

        // Create Qt timer to periodically process callbacks and check for timeout
        qt_timer_ = new QTimer(this);
        connect(qt_timer_, &QTimer::timeout, [this]() {
            // Process callbacks using our own executor
            if (executor_) {
                executor_->spin_some(std::chrono::milliseconds(0));
            }
            // Update display to check for timeout
            this->updateDisplay();
        });
        qt_timer_->start(100);  // 100ms interval
    }

    void ArucoIdDisplay::load(const rviz_common::Config &config) {
        rviz_common::Panel::load(config);
    }

    void ArucoIdDisplay::save(rviz_common::Config config) const {
        rviz_common::Panel::save(config);
    }

    void ArucoIdDisplay::processMessage(std_msgs::msg::Float32::ConstSharedPtr msg) {
        // Store the value and update last message time
        current_value_ = msg->data;
        last_message_time_ = clock_->now();
        // Debug output (can be removed later)
        RCLCPP_DEBUG(rclcpp::get_logger("aruco_id_display"), "Received message: %f", msg->data);
    }

    bool ArucoIdDisplay::isMessageDetected() const {
        if (last_message_time_.nanoseconds() == 0) {
            return false;
        }
        rclcpp::Time current_time = clock_->now();
        rclcpp::Duration time_since_last = current_time - last_message_time_;
        return time_since_last.seconds() < timeout_seconds_;
    }

    void ArucoIdDisplay::updateDisplay() {
        bool detected = isMessageDetected();
        
        if (detected) {
            status_label_->setText("Detected");
            status_label_->setStyleSheet("QLabel { font-size: 24px; font-weight: bold; padding: 20px; color: green; }");
            std::stringstream ss;
            ss << "ID: " << std::fixed << std::setprecision(1) << current_value_;
            id_label_->setText(QString::fromStdString(ss.str()));
        } else {
            status_label_->setText("None");
            status_label_->setStyleSheet("QLabel { font-size: 24px; font-weight: bold; padding: 20px; color: gray; }");
            id_label_->setText("ID: --");
        }
    }
} // namespace rviz_nav2

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_nav2::ArucoIdDisplay, rviz_common::Panel)
