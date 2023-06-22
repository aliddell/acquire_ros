#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "acquire.h"

class Runtime : public rclcpp::Node
{
public:
  Runtime() : Node("streamer"), runtime_{}
  {
  }

private:
  AcquireRuntime runtime_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char** argv)
{
  (void)argc;
  (void)argv;

  printf("hello world acquire package\n");
  return 0;
}
