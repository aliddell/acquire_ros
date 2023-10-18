#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <stdexcept>

#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/fill_image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

#include "acquire.h"
#include "device/hal/device.manager.h"

using namespace std::chrono_literals;

static std::string namespace_ = "acquire";

namespace {
size_t consumed_bytes(const VideoFrame *const cur,
                      const VideoFrame *const end) {
  return (uint8_t *)end - (uint8_t *)cur;
};

VideoFrame *next(VideoFrame *const cur) {
  return (VideoFrame *)(((uint8_t *)cur) + cur->bytes_of_frame);
}

void reporter(int is_error, const char *file, int line, const char *function,
              const char *msg) {
  auto logger = rclcpp::get_logger(namespace_ + ": inner");
  if (is_error) {
    RCLCPP_ERROR(logger, "%s(%d) - %s: %s\n", file, line, function, msg);
  } else {
    RCLCPP_DEBUG(logger, "%s(%d) - %s: %s\n", file, line, function, msg);
  }
}
}  // namespace

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

#define EXPECT(e, ...)                             \
  do {                                             \
    if (!(e)) {                                    \
      char buf[1 << 8] = {0};                      \
      ERR(__VA_ARGS__);                            \
      snprintf(buf, sizeof(buf) - 1, __VA_ARGS__); \
      throw std::runtime_error(buf);               \
    }                                              \
  } while (0)
#define CHECK(e) EXPECT(e, "Expression evaluated as false: %s", #e)
#define OK(e) CHECK(AcquireStatus_Ok == (e))
#define DEVOK(e) CHECK(Device_Ok == (e))

/// Check that a==b
/// example: `ASSERT_EQ(int,"%d",42,meaning_of_life())`
#define ASSERT_EQ(T, fmt, a, b)                                                \
  do {                                                                         \
    T a_ = (T)(a);                                                             \
    T b_ = (T)(b);                                                             \
    EXPECT(a_ == b_, "Expected %s == %s but " fmt " != " fmt, #a, #b, a_, b_); \
  } while (0)

#define SIZED(str) str, sizeof(str)

class AcquireStreamer final : public rclcpp::Node {
 public:
  AcquireStreamer()
      : Node("streamer"), runtime_{nullptr}, props_{0}, nframes_{0} {
    namespace_ = this->get_namespace();

    rcl_interfaces::msg::ParameterDescriptor descriptor{};
    descriptor.description = "Camera 0 identifier.";
    this->declare_parameter("camera0/identifier",
                            ".*simulated: uniform random.*", descriptor);

    descriptor.description = "Camera 1 identifier.";
    this->declare_parameter("camera1/identifier", "", descriptor);

    for (auto i = 0; i < 2; ++i) {
      const std::string camera_id = "camera" + std::to_string(i);
      const std::string camera_desc = "Camera " + std::to_string(i);

      descriptor.description = camera_desc + " topic.";
      this->declare_parameter(camera_id + "/topic",
                              "stream" + std::to_string(i), descriptor);

      descriptor.description = camera_desc + " binning.";
      this->declare_parameter(camera_id + "/binning", 1, descriptor);

      descriptor.description = camera_desc + " exposure time in microseconds.";
      this->declare_parameter(camera_id + "/exposure_time_us", 10000,
                              descriptor);

      descriptor.description = camera_desc + " image size.";
      this->declare_parameter(camera_id + "/image_size",
                              std::vector<int>{640, 480}, descriptor);
    }

    configure_publishers_();
    timer_ = this->create_wall_timer(
        5ms, std::bind(&AcquireStreamer::timer_callback, this));

    configure_streams();
    acquire_start(runtime_);
  }

  ~AcquireStreamer() {
    DEBUG("Destructor called.");
    if (runtime_) {
      acquire_abort(runtime_);
      acquire_shutdown(runtime_);
    }
  }

 private:
  AcquireRuntime *runtime_;
  AcquireProperties props_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::array<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr, 2>
      publishers_;

  uint64_t nframes_;

  void configure_stream(uint8_t stream) {
    const std::string camera = "camera" + std::to_string(stream);
    const std::string camera_id =
        this->get_parameter(camera + "/identifier").as_string();

    // don't configure a stream if no camera is specified
    if (camera_id.empty()) {
      return;
    }

    auto dm = acquire_device_manager(runtime_);
    CHECK(dm);

    AcquirePropertyMetadata meta = {0};
    OK(acquire_get_configuration_metadata(runtime_, &meta));

    // camera configuration
    DEVOK(device_manager_select(dm, DeviceKind_Camera, camera_id.c_str(),
                                camera_id.size(),
                                &props_.video[stream].camera.identifier));

    props_.video[stream].camera.settings.binning =
        this->get_parameter(camera + "/binning").as_int();
    props_.video[stream].camera.settings.pixel_type = SampleType_u8;
    props_.video[stream].camera.settings.exposure_time_us =
        this->get_parameter(camera + "/exposure_time_us").as_int();

    auto image_size = this->get_parameter(camera + "/image_size")
                          .get_value<std::vector<int>>();
    props_.video[stream].camera.settings.shape.x = image_size[0];
    props_.video[stream].camera.settings.shape.y = image_size[1];

    props_.video[stream].max_frame_count = UINT64_MAX;

    // storage configuration
    DEVOK(device_manager_select(dm, DeviceKind_Storage, SIZED("Trash") - 1,
                                &props_.video[stream].storage.identifier));
  }

  void configure_streams() {
    CHECK(runtime_ = acquire_init(reporter));

    OK(acquire_get_configuration(runtime_, &props_));

    configure_stream(0);
    configure_stream(1);

    OK(acquire_configure(runtime_, &props_));

    DEBUG("Configured.");
  }

  void send_data(uint8_t stream) {
    VideoFrame *beg, *end, *cur;
    OK(acquire_map_read(runtime_, stream, &beg, &end));
    DEBUG("stream %d got %zu frames", stream,
          (size_t)((uint8_t *)end - (uint8_t *)beg));
    for (cur = beg; cur < end; cur = next(cur)) {
      DEBUG("stream %d counting frame w id %lu", stream, cur->frame_id);
      ASSERT_EQ(uint32_t, "%u", cur->shape.dims.width,
                props_.video[stream].camera.settings.shape.x);
      ASSERT_EQ(uint32_t, "%u", cur->shape.dims.height,
                props_.video[stream].camera.settings.shape.y);

      sensor_msgs::msg::Image msg;
      CHECK(sensor_msgs::fillImage(
          msg, sensor_msgs::image_encodings::MONO8, cur->shape.dims.height,
          cur->shape.dims.width, cur->shape.strides.height, cur->data));

      DEBUG("Publishing: '%lu'", cur->frame_id);
      publishers_.at(stream)->publish(msg);
      ++nframes_;
    }

    auto n = (uint32_t)consumed_bytes(beg, end);
    OK(acquire_unmap_read(runtime_, stream, n));
    if (n) DEBUG("stream %d consumed bytes %d", stream, n);
  }

  void timer_callback() {
    if (DeviceState_Running != acquire_get_state(runtime_)) {
      return;
    }

    send_data(0);
    send_data(1);
  }

  void configure_publishers_() {
    rclcpp::QoS qos(rclcpp::KeepAll(), rmw_qos_profile_sensor_data);
    publishers_.at(0) = this->create_publisher<sensor_msgs::msg::Image>(
        this->get_parameter("camera0/topic").as_string(), qos);
    publishers_.at(1) = this->create_publisher<sensor_msgs::msg::Image>(
        this->get_parameter("camera1/topic").as_string(), qos);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto streamer = std::make_shared<AcquireStreamer>();
  rclcpp::spin(streamer);
  rclcpp::shutdown();

  return 0;
}
