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
  AcquireViewer() : Node("viewer"), window_name_{ "acquire viewer" }
  {
    rclcpp::QoS qos(rclcpp::KeepAll(), rmw_qos_profile_sensor_data);
    cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE | cv::WINDOW_KEEPRATIO | cv::WINDOW_GUI_EXPANDED);
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "microscope", qos, std::bind(&AcquireViewer::topic_callback, this, std::placeholders::_1));
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  std::string window_name_;

  void topic_callback(const sensor_msgs::msg::Image& msg)
  {
    DEBUG("Got an image: %u x %u, %lu bytes", msg.width, msg.height, msg.data.size());
    cv::Mat img(msg.height, msg.width, CV_8U, (void*)msg.data.data());
    cv::imshow(window_name_, img);
    cv::waitKey(10);
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