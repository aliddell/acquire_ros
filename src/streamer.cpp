#include "rclcpp/rclcpp.hpp"
#include <cstdio>

class Runtime
{
public:
  Runtime() {

  }

private:
  int foo_;
};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world acquire package\n");
  return 0;
}
