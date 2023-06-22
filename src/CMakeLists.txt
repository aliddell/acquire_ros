aq_require(acquire-driver-common)
aq_require(acquire-video-runtime)

add_executable(streamer streamer.cpp)
ament_target_dependencies(streamer
                          rclcpp
                          std_msgs
                          sensor_msgs)
target_include_directories(streamer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(streamer PUBLIC c_std_11 cxx_std_20)  # Require C11 and C++20

target_link_libraries(streamer
    acquire-core-logger
    acquire-core-platform
    acquire-device-kit
    acquire-device-properties
    acquire-video-runtime
)

install(TARGETS streamer
  DESTINATION lib/${PROJECT_NAME})
