#include <cstdio>
#include <chrono>
#include <functional>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "acquire.h"
#include "device/hal/device.manager.h"

using namespace std::chrono_literals;

namespace
{
std::shared_ptr<rclcpp::Node> streamer;

VideoFrame* next(VideoFrame* const cur)
{
  return (VideoFrame*)(((uint8_t*)cur) + cur->bytes_of_frame);
}

void reporter(int is_error, const char* file, int line, const char* function, const char* msg)
{
  fprintf(is_error ? stderr : stdout, "%s%s(%d) - %s: %s\n", is_error ? "ERROR " : "", file, line, function, msg);
}

void aq_logger(int is_error, const char* file, int line, const char* function, const char* fmt, ...)
{
  char buf[1024] = { 0 };
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);  // NOLINT
  va_end(ap);

  if (is_error)
  {
    RCLCPP_ERROR(streamer->get_logger(), "ERROR %s(%d) - %s: %s\n", file, line, function, buf);
  }
  else
  {
    RCLCPP_INFO(streamer->get_logger(), "%s(%d) - %s: %s\n", file, line, function, buf);
  }
}
}  // namespace

#define L aq_logger
#define LOG(...) L(0, __FILE__, __LINE__, __FUNCTION__, __VA_ARGS__)
#define ERR(...) L(1, __FILE__, __LINE__, __FUNCTION__, __VA_ARGS__)
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
    LOG("Destructor called");
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

    AcquireProperties props = { 0 };
    OK(acquire_get_configuration(runtime_, &props));

    DEVOK(device_manager_select(dm, DeviceKind_Camera, SIZED("simulated.*random.*") - 1,
                                &props.video[0].camera.identifier));
    DEVOK(device_manager_select(dm, DeviceKind_Storage, SIZED("Trash") - 1, &props.video[0].storage.identifier));

    OK(acquire_configure(runtime_, &props));

    AcquirePropertyMetadata metadata = { 0 };
    OK(acquire_get_configuration_metadata(runtime_, &metadata));

    props.video[0].camera.settings.binning = 1;
    props.video[0].camera.settings.pixel_type = SampleType_u8;
    props.video[0].camera.settings.shape = {
      .x = 64,
      .y = 48,
    };
    props.video[0].camera.settings.exposure_time_us = 1e4;
    props.video[0].max_frame_count = 0;

    OK(acquire_configure(runtime_, &props));
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
      LOG("stream %d counting frame w id %d", 0, cur->frame_id);
      CHECK(cur->shape.dims.width == props_.video[0].camera.settings.shape.x);
      CHECK(cur->shape.dims.height == props_.video[0].camera.settings.shape.y);
      ++nframes_;
    }
    sensor_msgs::msg::Image msg;
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  streamer = std::make_shared<AcquireStreamer>();
  rclcpp::spin(streamer);
  rclcpp::shutdown();

  return 0;
}
