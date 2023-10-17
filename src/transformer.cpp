#include <array>
#include <functional>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/fill_image.hpp"

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

#define EXPECT(e, ...)                                                                                                 \
  do                                                                                                                   \
  {                                                                                                                    \
    if (!(e))                                                                                                          \
    {                                                                                                                  \
      char buf[1 << 8] = { 0 };                                                                                        \
      ERR(__VA_ARGS__);                                                                                                \
      snprintf(buf, sizeof(buf) - 1, __VA_ARGS__);                                                                     \
      throw std::runtime_error(buf);                                                                                   \
    }                                                                                                                  \
  } while (0)
#define CHECK(e) EXPECT(e, "Expression evaluated as false: %s", #e)

class AcquireTransformer : public rclcpp::Node
{
public:
  AcquireTransformer() : Node("transformer")
  {
    rcl_interfaces::msg::ParameterDescriptor keep_last_desc{};
    keep_last_desc.description = "Number of frames to keep.";
    this->declare_parameter("keep_last", -1, keep_last_desc);
    
    rcl_interfaces::msg::ParameterDescriptor input_topic_desc{};
    this->declare_parameter("input_topic", "stream0", input_topic_desc);

    rcl_interfaces::msg::ParameterDescriptor output_topic_desc{};
    this->declare_parameter("output_topic", "stream0_transformed", output_topic_desc);

    configure_subscriber_();
    configure_publisher_();
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

  void transform_image(sensor_msgs::msg::Image::SharedPtr msg)
  {
    DEBUG("Got an image: %u x %u, %lu bytes", msg->width, msg->height, msg->data.size());
    cv::Mat img(msg->height, msg->width, CV_8U, (void*)msg->data.data());
    img = 255 - img;
    
    sensor_msgs::msg::Image out_msg;
    CHECK(sensor_msgs::fillImage(out_msg, "mono8", msg->height, msg->width, msg->width, img.data));

    publisher_->publish(out_msg);
  }
  
  void configure_subscriber_()
   {
    auto keep_last = this->get_parameter("keep_last").as_int();

    std::shared_ptr<rclcpp::QoS> qos;
    if (keep_last >= 0) {
      keep_last = std::max((int64_t)1, keep_last);
      qos = std::make_shared<rclcpp::QoS>(rclcpp::KeepLast(keep_last), rmw_qos_profile_sensor_data);
    } else {
      qos = std::make_shared<rclcpp::QoS>(rclcpp::KeepAll(), rmw_qos_profile_sensor_data);
    }

    const auto topic = this->get_parameter("input_topic").as_string();
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        topic, *qos, std::bind(&AcquireTransformer::transform_image, this, std::placeholders::_1));
   }

  void configure_publisher_()
  {
    auto keep_last = this->get_parameter("keep_last").as_int();

    std::shared_ptr<rclcpp::QoS> qos;
    if (keep_last >= 0) {
      keep_last = std::max((int64_t)1, keep_last);
      qos = std::make_shared<rclcpp::QoS>(rclcpp::KeepLast(keep_last), rmw_qos_profile_sensor_data);
    } else {
      qos = std::make_shared<rclcpp::QoS>(rclcpp::KeepAll(), rmw_qos_profile_sensor_data);
    }

    const auto topic = this->get_parameter("output_topic").as_string();
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(topic, *qos);
   }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto viewer = std::make_shared<AcquireTransformer>();
  rclcpp::spin(viewer);
  rclcpp::shutdown();

  return 0;
}