#include <cstdio>
#include <cstdint>
#include <chrono>
#include <functional>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "acquire.h"
#include "device/hal/device.manager.h"

using namespace std::chrono_literals;

class AcquireStreamer;

namespace
{
VideoFrame* next(VideoFrame* const cur)
{
  return (VideoFrame*)(((uint8_t*)cur) + cur->bytes_of_frame);
}

void reporter(int is_error, const char* file, int line, const char* function, const char* msg)
{
  auto logger = rclcpp::get_logger("acquire_inner");
  if (is_error)
  {
    RCLCPP_ERROR(logger, "%s(%d) - %s: %s\n", file, line, function, msg);
  }
  else
  {
    RCLCPP_INFO(logger, "%s(%d) - %s: %s\n", file, line, function, msg);
  }
}
}  // namespace

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
#define OK(e) CHECK(AcquireStatus_Ok == (e))
#define DEVOK(e) CHECK(Device_Ok == (e))

/// Check that a==b
/// example: `ASSERT_EQ(int,"%d",42,meaning_of_life())`
#define ASSERT_EQ(T, fmt, a, b)                                                                                        \
  do                                                                                                                   \
  {                                                                                                                    \
    T a_ = (T)(a);                                                                                                     \
    T b_ = (T)(b);                                                                                                     \
    EXPECT(a_ == b_, "Expected %s == %s but " fmt " != " fmt, #a, #b, a_, b_);                                         \
  } while (0)

#define SIZED(str) str, sizeof(str)

class AcquireStreamer : public rclcpp::Node
{
public:
  AcquireStreamer() : Node("streamer"), runtime_{ nullptr }, props_{ 0 }, nframes_{ 0 }
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("microscope", 10);
    timer_ = this->create_wall_timer(10ms, std::bind(&AcquireStreamer::timer_callback, this));
    configure_stream();
    acquire_start(runtime_);
  }

  ~AcquireStreamer()
  {
    DEBUG("Destructor called.");
    if (runtime_)
    {
      acquire_abort(runtime_);
      acquire_shutdown(runtime_);
    }
  }

private:
  AcquireRuntime* runtime_;
  AcquireProperties props_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

  uint64_t nframes_;

  void configure_stream()
  {
    CHECK(runtime_ = acquire_init(reporter));

    auto dm = acquire_device_manager(runtime_);
    CHECK(dm);

    OK(acquire_get_configuration(runtime_, &props_));

    DEVOK(device_manager_select(dm, DeviceKind_Camera, SIZED("simulated.*random.*") - 1,
                                &props_.video[0].camera.identifier));
    DEVOK(device_manager_select(dm, DeviceKind_Storage, SIZED("Trash") - 1, &props_.video[0].storage.identifier));

    props_.video[0].camera.settings.binning = 1;
    props_.video[0].camera.settings.pixel_type = SampleType_u8;
    props_.video[0].camera.settings.shape = {
      .x = 64,
      .y = 48,
    };
    props_.video[0].camera.settings.exposure_time_us = 1e3;
    props_.video[0].max_frame_count = UINT64_MAX;

    OK(acquire_configure(runtime_, &props_));

    DEBUG("Configured.");
  }

  void timer_callback()
  {
    if (DeviceState_Running != acquire_get_state(runtime_))
    {
      return;
    }

    VideoFrame *beg, *end, *cur;
    OK(acquire_map_read(runtime_, 0, &beg, &end));
    for (cur = beg; cur < end; cur = next(cur))
    {
      DEBUG("stream %d counting frame w id %d", 0, cur->frame_id);
      ASSERT_EQ(uint32_t, "%lu", cur->shape.dims.width, props_.video[0].camera.settings.shape.x);
      ASSERT_EQ(uint32_t, "%lu", cur->shape.dims.height, props_.video[0].camera.settings.shape.y);
      ++nframes_;
    }
    sensor_msgs::msg::Image msg;
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto streamer = std::make_shared<AcquireStreamer>();
  rclcpp::spin(streamer);
  rclcpp::shutdown();

  return 0;
}
