#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "acquire.h"

using namespace std::chrono_literals;

class AcquireStreamer : public rclcpp::Node
{
public:
  AcquireStreamer() : Node("streamer"), runtime_{}
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("microscope", 10);
  }

private:
  AcquireRuntime runtime_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AcquireStreamer>());
  rclcpp::shutdown();

  return 0;
}
