#include <cstdio>
#include <cstdint>
#include <chrono>
#include <functional>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/fill_image.hpp"

#include "acquire.h"
#include "device/hal/device.manager.h"

using namespace std::chrono_literals;

namespace
{
size_t consumed_bytes(const VideoFrame* const cur, const VideoFrame* const end)
{
  return (uint8_t*)end - (uint8_t*)cur;
};

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
    RCLCPP_DEBUG(logger, "%s(%d) - %s: %s\n", file, line, function, msg);
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
    rclcpp::QoS qos(rclcpp::KeepAll(), rmw_qos_profile_sensor_data);
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("microscope", qos);
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
      .x = 1280,
      .y = 720,
    };
    props_.video[0].camera.settings.exposure_time_us = 1e4;
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
    DEBUG("stream %d got %zu frames", 0, (size_t)((uint8_t*)end - (uint8_t*)beg));
    for (cur = beg; cur < end; cur = next(cur))
    {
      DEBUG("stream %d counting frame w id %lu", 0, cur->frame_id);
      ASSERT_EQ(uint32_t, "%u", cur->shape.dims.width, props_.video[0].camera.settings.shape.x);
      ASSERT_EQ(uint32_t, "%u", cur->shape.dims.height, props_.video[0].camera.settings.shape.y);

      sensor_msgs::msg::Image msg;
      CHECK(sensor_msgs::fillImage(msg, sensor_msgs::image_encodings::MONO8, cur->shape.dims.height,
                                   cur->shape.dims.width, cur->shape.strides.height, cur->data));

      DEBUG("Publishing: '%lu'", cur->frame_id);
      publisher_->publish(msg);
      ++nframes_;
    }

    auto n = (uint32_t)consumed_bytes(beg, end);
    OK(acquire_unmap_read(runtime_, 0, n));
    if (n)
      DEBUG("stream %d consumed bytes %d", 0, n);
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
