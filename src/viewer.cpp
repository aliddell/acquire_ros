#include <array>
#include <functional>

#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"

#define DEBUG(...)                                                     \
  do {                                                                 \
    char args[1 << 8] = {0};                                           \
    snprintf(args, sizeof(args) - 1, __VA_ARGS__);                     \
    RCLCPP_DEBUG(get_logger(), "%s (%d) - %s: %s", __FILE__, __LINE__, \
                 __FUNCTION__, args);                                  \
  } while (0)
#define LOG(...)                                                      \
  do {                                                                \
    char args[1 << 8] = {0};                                          \
    snprintf(args, sizeof(args) - 1, __VA_ARGS__);                    \
    RCLCPP_INFO(get_logger(), "%s (%d) - %s: %s", __FILE__, __LINE__, \
                __FUNCTION__, args);                                  \
  } while (0)
#define WARN(...)                                                     \
  do {                                                                \
    char args[1 << 8] = {0};                                          \
    snprintf(args, sizeof(args) - 1, __VA_ARGS__);                    \
    RCLCPP_WARN(get_logger(), "%s (%d) - %s: %s", __FILE__, __LINE__, \
                __FUNCTION__, args);                                  \
  } while (0)
#define ERR(...)                                                       \
  do {                                                                 \
    char args[1 << 8] = {0};                                           \
    snprintf(args, sizeof(args) - 1, __VA_ARGS__);                     \
    RCLCPP_ERROR(get_logger(), "%s (%d) - %s: %s", __FILE__, __LINE__, \
                 __FUNCTION__, args);                                  \
  } while (0)

class AcquireViewer : public rclcpp::Node {
 public:
  AcquireViewer() : Node("viewer") {
    rcl_interfaces::msg::ParameterDescriptor keep_last_desc{};
    keep_last_desc.description = "Number of frames to keep.";
    this->declare_parameter("keep_last", -1, keep_last_desc);

    rcl_interfaces::msg::ParameterDescriptor topic_desc{};
    this->declare_parameter("topic", "stream0", topic_desc);

    stream_topic_ = this->get_parameter("topic").as_string();
    cv::namedWindow(window_name(), cv::WINDOW_AUTOSIZE | cv::WINDOW_KEEPRATIO |
                                       cv::WINDOW_GUI_EXPANDED);

    configure_subscriber_();
  }

 private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  std::string stream_topic_;

  void display_image(sensor_msgs::msg::Image::SharedPtr msg) {
    DEBUG("Got an image: %u x %u, %lu bytes", msg->width, msg->height,
          msg->data.size());
    cv::Mat img(msg->height, msg->width, CV_8U, (void *)msg->data.data());
    cv::imshow(window_name(), img);
    cv::waitKey(1);
  }

  void configure_subscriber_() {
    auto keep_last = this->get_parameter("keep_last").as_int();

    std::shared_ptr<rclcpp::QoS> qos;
    if (keep_last >= 0) {
      keep_last = std::max((int64_t)1, keep_last);
      qos = std::make_shared<rclcpp::QoS>(rclcpp::KeepLast(keep_last),
                                          rmw_qos_profile_sensor_data);
    } else {
      qos = std::make_shared<rclcpp::QoS>(rclcpp::KeepAll(),
                                          rmw_qos_profile_sensor_data);
    }

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        stream_topic_, *qos,
        std::bind(&AcquireViewer::display_image, this, std::placeholders::_1));
  }

  std::string window_name() const {
    return std::string(get_namespace()) + ": " + stream_topic_;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto viewer = std::make_shared<AcquireViewer>();
  rclcpp::spin(viewer);
  rclcpp::shutdown();

  return 0;
}