#ifndef KINECT_ROS2__KINECT_ROS2_COMPONENT_HPP_
#define KINECT_ROS2__KINECT_ROS2_COMPONENT_HPP_

extern "C"
{
  #include "libfreenect/libfreenect.h"
}
#include <chrono>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "camera_info_manager/camera_info_manager.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/float64.hpp"
#include "cv_bridge/cv_bridge.h"

namespace kinect_ros2
{

/* Kinect v1 (Xbox 360) driver: RGB and depth frame pairs with one stamp, published at
 * `publish_rate_hz`, depth registered to the RGB image when `depth_registered`, and the tilt motor
 * driven by the `tilt_deg` parameter (measured angle on `~/tilt_state`). Waits `device_wait_s` for
 * the Kinect to appear on the USB bus and throws when it does not, so a launch with respawn
 * retries instead of a process that segfaults on a null device. */
class KinectRosComponent : public rclcpp::Node
{
public:
  explicit KinectRosComponent(const rclcpp::NodeOptions & options);
  ~KinectRosComponent();

private:
  freenect_context * fn_ctx_ = nullptr;
  freenect_device * fn_dev_ = nullptr;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr tilt_timer_;

  std::shared_ptr<camera_info_manager::CameraInfoManager> rgb_info_manager_, depth_info_manager_;
  sensor_msgs::msg::CameraInfo rgb_info_, depth_info_;
  std::string depth_frame_id_ = "kinect_depth";

  image_transport::CameraPublisher depth_pub_, rgb_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr tilt_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_handle_;

  bool depth_registered_ = true;
  double publish_rate_hz_ = 5.0;
  double device_wait_s_ = 20.0;
  std::mutex tilt_mutex_;
  bool tilt_pending_ = false;
  double tilt_target_deg_ = 0.0;

  bool published_once_ = false;
  std::chrono::steady_clock::time_point last_publish_;
  std::chrono::steady_clock::time_point pair_wait_start_;

  static void depth_cb(freenect_device * dev, void * depth_ptr, uint32_t timestamp);
  static void rgb_cb(freenect_device * dev, void * rgb_ptr, uint32_t timestamp);

  void open_device();
  void fail(const std::string & what);
  void timer_callback();
  void apply_tilt();
  void tilt_timer_callback();
  rcl_interfaces::msg::SetParametersResult on_set_parameters(const std::vector<rclcpp::Parameter> & parameters);
};

}
#endif  // KINECT_ROS2__KINECT_ROS2_COMPONENT_HPP_
