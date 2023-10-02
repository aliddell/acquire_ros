#include <array>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"

#define DEBUG(...)                                                                                                     \
  do                                                                                                                   \
  {                                                                                                                    \
    char args[1 << 8] = { 0 };                                                                                         \
    snprintf(args, sizeof(args) - 1, __VA_ARGS__);                                                                     \
    RCLCPP_DEBUG(get_logger(), "%s (%d) - %s: %s", __FILE__, __LINE__, __FUNCTION__, args);                            \
  } while (0)
#define LOG(...)                                                                                                       \
  do                                                                                                                   \
  {                                                                                                                    \
    char args[1 << 8] = { 0 };                                                                                         \
    snprintf(args, sizeof(args) - 1, __VA_ARGS__);                                                                     \
    RCLCPP_INFO(get_logger(), "%s (%d) - %s: %s", __FILE__, __LINE__, __FUNCTION__, args);                             \
  } while (0)
#define WARN(...)                                                                                                      \
  do                                                                                                                   \
  {                                                                                                                    \
    char args[1 << 8] = { 0 };                                                                                         \
    snprintf(args, sizeof(args) - 1, __VA_ARGS__);                                                                     \
    RCLCPP_WARN(get_logger(), "%s (%d) - %s: %s", __FILE__, __LINE__, __FUNCTION__, args);                             \
  } while (0)
#define ERR(...)                                                                                                       \
  do                                                                                                                   \
  {                                                                                                                    \
    char args[1 << 8] = { 0 };                                                                                         \
    snprintf(args, sizeof(args) - 1, __VA_ARGS__);                                                                     \
    RCLCPP_ERROR(get_logger(), "%s (%d) - %s: %s", __FILE__, __LINE__, __FUNCTION__, args);                            \
  } while (0)

class AcquireViewer : public rclcpp::Node
{
public:
  AcquireViewer() : Node("viewer")
  {
    rcl_interfaces::msg::ParameterDescriptor keep_last_desc{};
    keep_last_desc.description = "Number of frames to keep.";
    this->declare_parameter("keep_last", -1, keep_last_desc);

    window_names_ = { "stream 0", "stream 1" };
    cv::namedWindow(window_names_.at(0), cv::WINDOW_AUTOSIZE | cv::WINDOW_KEEPRATIO | cv::WINDOW_GUI_EXPANDED);
    cv::namedWindow(window_names_.at(1), cv::WINDOW_AUTOSIZE | cv::WINDOW_KEEPRATIO | cv::WINDOW_GUI_EXPANDED);

    configure_subscribers_();
  }

private:
  std::array<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr, 2> subscriptions_;
  std::array<std::string, 2> window_names_;

  void topic_callback(int stream, sensor_msgs::msg::Image::SharedPtr msg)
  {
    DEBUG("Got an image: %u x %u, %lu bytes", msg->width, msg->height, msg->data.size());
    cv::Mat img(msg->height, msg->width, CV_8U, (void*)msg->data.data());
    cv::imshow(window_names_.at(stream), img);
    cv::waitKey(10);
  }

  void stream0(sensor_msgs::msg::Image::SharedPtr msg)
  {
    topic_callback(0, msg);
  }

  void stream1(sensor_msgs::msg::Image::SharedPtr msg)
  {
    topic_callback(1, msg);
  }

   void configure_subscribers_()
   {
    auto keep_last = this->get_parameter("keep_last").as_int();

    std::shared_ptr<rclcpp::QoS> qos;
    if (keep_last >= 0) {
      keep_last = std::max((int64_t)1, keep_last);
      qos = std::make_shared<rclcpp::QoS>(rclcpp::KeepLast(keep_last), rmw_qos_profile_sensor_data);
    } else {
      qos = std::make_shared<rclcpp::QoS>(rclcpp::KeepAll(), rmw_qos_profile_sensor_data);
    }

    DEBUG("keep_last: %d", keep_last);
    subscriptions_.at(0) = this->create_subscription<sensor_msgs::msg::Image>(
        "stream0", *qos, std::bind(&AcquireViewer::stream0, this, std::placeholders::_1));
    subscriptions_.at(1) = this->create_subscription<sensor_msgs::msg::Image>(
        "stream1", *qos, std::bind(&AcquireViewer::stream1, this, std::placeholders::_1));
   } 
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto viewer = std::make_shared<AcquireViewer>();
  rclcpp::spin(viewer);
  rclcpp::shutdown();

  return 0;
}