#include "kinect_ros2/kinect_ros2_component.hpp"

#include <stdexcept>
#include <thread>

#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

namespace kinect_ros2
{
/* libfreenect writes each frame into a static buffer of its own; the cv::Mats wrap that memory so
nothing is copied, and the callbacks only raise a flag. The callbacks run on the timer thread, inside
freenect_process_events, so the flags need no lock. A flag stays up until the frame is published (or
dropped by the rate limit), which also stops the next frame from overwriting the buffer meanwhile. */
static cv::Mat _depth_image(cv::Mat::zeros(cv::Size(640, 480), CV_16UC1));
static cv::Mat _rgb_image(cv::Mat::zeros(cv::Size(640, 480), CV_8UC3));

static uint16_t * _freenect_depth_pointer = nullptr;
static uint8_t * _freenect_rgb_pointer = nullptr;

static bool _depth_flag;
static bool _rgb_flag;

static constexpr double TILT_MIN_DEG = -30.0;
static constexpr double TILT_MAX_DEG = 30.0;
static constexpr auto PAIR_WAIT = std::chrono::milliseconds(500);  // publish a lone frame after this

static rcl_interfaces::msg::ParameterDescriptor describe(const std::string & text, bool read_only)
{
  rcl_interfaces::msg::ParameterDescriptor desc;
  desc.description = text;
  desc.read_only = read_only;
  return desc;
}

KinectRosComponent::KinectRosComponent(const rclcpp::NodeOptions & options)
: Node("kinect_ros2", options)
{
  rcl_interfaces::msg::ParameterDescriptor tilt_desc = describe(
    "Tilt motor angle in degrees above level (-30..30); gpt-pet-mcp's do_look sets 27 or 0.", false);
  rcl_interfaces::msg::FloatingPointRange tilt_range;
  tilt_range.from_value = TILT_MIN_DEG;
  tilt_range.to_value = TILT_MAX_DEG;
  tilt_range.step = 0.0;
  tilt_desc.floating_point_range.push_back(tilt_range);

  depth_registered_ = declare_parameter<bool>(
    "depth_registered", true,
    describe("Depth aligned to the RGB image (FREENECT_DEPTH_REGISTERED, millimetres, published with the RGB "
             "camera's intrinsics and frame); false: raw depth (FREENECT_DEPTH_MM) in the depth camera's frame.", true));
  device_wait_s_ = declare_parameter<double>(
    "device_wait_s", 20.0, describe("Seconds to wait for a Kinect on the USB bus before giving up.", true));
  publish_rate_hz_ = declare_parameter<double>(
    "publish_rate_hz", 5.0, describe("Frame pairs published per second; 0 publishes every frame the Kinect delivers.", false));
  tilt_target_deg_ = declare_parameter<double>("tilt_deg", 0.0, tilt_desc);
  depth_frame_id_ = depth_registered_ ? "kinect_rgb" : "kinect_depth";

  std::string pkg_share = ament_index_cpp::get_package_share_directory("kinect_ros2");

  depth_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
    this, "kinect",
    "file://" + pkg_share + "/cfg/calibration_depth.yaml");

  rgb_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
    this, "kinect",
    "file://" + pkg_share + "/cfg/calibration_rgb.yaml");

  rgb_info_ = rgb_info_manager_->getCameraInfo();
  rgb_info_.header.frame_id = "kinect_rgb";
  depth_info_ = depth_info_manager_->getCameraInfo();
  depth_info_.header.frame_id = depth_frame_id_;

  depth_pub_ = image_transport::create_camera_publisher(this, "depth/image_raw");
  rgb_pub_ = image_transport::create_camera_publisher(this, "image_raw");
  tilt_pub_ = create_publisher<std_msgs::msg::Float64>("tilt_state", 10);

  open_device();

  {
    std::lock_guard<std::mutex> lock(tilt_mutex_);
    tilt_pending_ = true;  // level (or the configured angle) at start
  }
  param_handle_ = add_on_set_parameters_callback(
    std::bind(&KinectRosComponent::on_set_parameters, this, std::placeholders::_1));
  timer_ = create_wall_timer(1ms, std::bind(&KinectRosComponent::timer_callback, this));
  tilt_timer_ = create_wall_timer(500ms, std::bind(&KinectRosComponent::tilt_timer_callback, this));

  RCLCPP_INFO(
    get_logger(), "Kinect streaming: %s depth, %.1f pairs/s, tilt %.0f deg",
    depth_registered_ ? "registered" : "raw", publish_rate_hz_, tilt_target_deg_);
}

KinectRosComponent::~KinectRosComponent()
{
  RCLCPP_INFO(get_logger(), "stopping the Kinect");
  if (fn_dev_ != nullptr) {
    freenect_stop_depth(fn_dev_);
    freenect_stop_video(fn_dev_);
    freenect_close_device(fn_dev_);
    fn_dev_ = nullptr;
  }
  if (fn_ctx_ != nullptr) {
    freenect_shutdown(fn_ctx_);
    fn_ctx_ = nullptr;
  }
}

void KinectRosComponent::fail(const std::string & what)
{
  RCLCPP_ERROR(get_logger(), "FREENECT - %s", what.c_str());
  if (fn_dev_ != nullptr) {
    freenect_close_device(fn_dev_);
    fn_dev_ = nullptr;
  }
  if (fn_ctx_ != nullptr) {
    freenect_shutdown(fn_ctx_);
    fn_ctx_ = nullptr;
  }
  throw std::runtime_error("kinect_ros2: " + what);
}

void KinectRosComponent::open_device()
{
  if (freenect_init(&fn_ctx_, NULL) < 0) {
    fn_ctx_ = nullptr;
    fail("freenect_init failed");
  }
  freenect_set_log_level(fn_ctx_, FREENECT_LOG_FATAL);
  // camera and motor only: no audio subdevice, so no firmware upload re-enumerates the shared USB hub
  freenect_select_subdevices(
    fn_ctx_, static_cast<freenect_device_flags>(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(device_wait_s_);
  int num_devices = freenect_num_devices(fn_ctx_);
  while (num_devices == 0 && rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "FREENECT - no Kinect on the USB bus yet, waiting up to %.0f s",
      device_wait_s_);
    std::this_thread::sleep_for(500ms);
    num_devices = freenect_num_devices(fn_ctx_);
  }
  if (num_devices < 0) {
    fail("freenect_num_devices failed");
  }
  if (num_devices == 0) {
    fail("no Kinect found within device_wait_s");
  }
  if (freenect_open_device(fn_ctx_, &fn_dev_, 0) < 0) {
    fn_dev_ = nullptr;
    fail("freenect_open_device failed");
  }
  const freenect_depth_format depth_format = depth_registered_ ? FREENECT_DEPTH_REGISTERED : FREENECT_DEPTH_MM;
  if (freenect_set_depth_mode(fn_dev_, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, depth_format)) < 0) {
    fail("set depth mode failed");
  }
  if (freenect_set_video_mode(
      fn_dev_, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB)) < 0)
  {
    fail("set video mode failed");
  }
  freenect_set_depth_callback(fn_dev_, depth_cb);
  freenect_set_video_callback(fn_dev_, rgb_cb);
  if (freenect_start_depth(fn_dev_) < 0) {
    fail("start depth failed");
  }
  if (freenect_start_video(fn_dev_) < 0) {
    fail("start video failed");
  }
}

void KinectRosComponent::depth_cb(freenect_device * dev, void * depth_ptr, uint32_t timestamp)
{
  (void)dev;
  (void)timestamp;
  if (_depth_flag) {
    return;
  }
  if (_freenect_depth_pointer != (uint16_t *)depth_ptr) {
    _depth_image = cv::Mat(480, 640, CV_16UC1, depth_ptr);
    _freenect_depth_pointer = (uint16_t *)depth_ptr;
  }
  _depth_flag = true;
}

void KinectRosComponent::rgb_cb(freenect_device * dev, void * rgb_ptr, uint32_t timestamp)
{
  (void)dev;
  (void)timestamp;
  if (_rgb_flag) {
    return;
  }
  if (_freenect_rgb_pointer != (uint8_t *)rgb_ptr) {
    _rgb_image = cv::Mat(480, 640, CV_8UC3, rgb_ptr);
    _freenect_rgb_pointer = (uint8_t *)rgb_ptr;
  }
  _rgb_flag = true;
}

void KinectRosComponent::timer_callback()
{
  int ret = freenect_process_events(fn_ctx_);
  if (ret < 0) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "freenect_process_events error: %d", ret);
  }
  apply_tilt();

  const auto now_steady = std::chrono::steady_clock::now();
  if (!_depth_flag && !_rgb_flag) {
    pair_wait_start_ = std::chrono::steady_clock::time_point{};
    return;
  }
  if (!(_depth_flag && _rgb_flag)) {
    // half a pair: give its partner PAIR_WAIT to arrive, then publish what there is
    if (pair_wait_start_ == std::chrono::steady_clock::time_point{}) {
      pair_wait_start_ = now_steady;
      return;
    }
    if (now_steady - pair_wait_start_ < PAIR_WAIT) {
      return;
    }
  }
  pair_wait_start_ = std::chrono::steady_clock::time_point{};

  if (publish_rate_hz_ > 0.0 && published_once_ &&
    now_steady - last_publish_ < std::chrono::duration<double>(1.0 / publish_rate_hz_))
  {
    _depth_flag = false;  // drop this pair; the buffers take the next one
    _rgb_flag = false;
    return;
  }

  const auto stamp = now();  // one stamp for the pair: subscribers can match them exactly

  if (_depth_flag) {
    auto depth_header = std_msgs::msg::Header();
    depth_header.frame_id = depth_frame_id_;
    depth_header.stamp = stamp;
    // registered depth lives in the RGB camera's frame and has its intrinsics
    sensor_msgs::msg::CameraInfo depth_info = depth_registered_ ? rgb_info_ : depth_info_;
    depth_info.header.frame_id = depth_frame_id_;
    depth_info.header.stamp = stamp;
    auto msg = cv_bridge::CvImage(depth_header, "16UC1", _depth_image).toImageMsg();
    depth_pub_.publish(*msg, depth_info);
    _depth_flag = false;
  }

  if (_rgb_flag) {
    auto rgb_header = std_msgs::msg::Header();
    rgb_header.frame_id = "kinect_rgb";
    rgb_header.stamp = stamp;
    rgb_info_.header.stamp = stamp;
    auto msg = cv_bridge::CvImage(rgb_header, "rgb8", _rgb_image).toImageMsg();
    rgb_pub_.publish(*msg, rgb_info_);
    _rgb_flag = false;
  }

  last_publish_ = now_steady;
  published_once_ = true;
}

void KinectRosComponent::apply_tilt()
{
  double target = 0.0;
  bool pending = false;
  {
    std::lock_guard<std::mutex> lock(tilt_mutex_);
    pending = tilt_pending_;
    target = tilt_target_deg_;
    tilt_pending_ = false;
  }
  if (!pending || fn_dev_ == nullptr) {
    return;
  }
  if (freenect_set_tilt_degs(fn_dev_, target) < 0) {
    RCLCPP_WARN(get_logger(), "tilt to %.0f deg failed (is the motor subdevice open?)", target);
    return;
  }
  RCLCPP_INFO(get_logger(), "tilt -> %.0f deg", target);
}

void KinectRosComponent::tilt_timer_callback()
{
  if (fn_dev_ == nullptr) {
    return;
  }
  if (freenect_update_tilt_state(fn_dev_) < 0) {
    return;
  }
  freenect_raw_tilt_state * state = freenect_get_tilt_state(fn_dev_);
  if (state == nullptr) {
    return;
  }
  std_msgs::msg::Float64 msg;
  msg.data = freenect_get_tilt_degs(state);
  tilt_pub_->publish(msg);
}

rcl_interfaces::msg::SetParametersResult KinectRosComponent::on_set_parameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & parameter : parameters) {
    if (parameter.get_name() == "tilt_deg") {
      const double deg = parameter.as_double();
      if (deg < TILT_MIN_DEG || deg > TILT_MAX_DEG) {
        result.successful = false;
        result.reason = "tilt_deg must be within -30..30 degrees";
        return result;
      }
      std::lock_guard<std::mutex> lock(tilt_mutex_);
      tilt_target_deg_ = deg;
      tilt_pending_ = true;
    } else if (parameter.get_name() == "publish_rate_hz") {
      const double hz = parameter.as_double();
      if (hz < 0.0) {
        result.successful = false;
        result.reason = "publish_rate_hz must be 0 or positive";
        return result;
      }
      publish_rate_hz_ = hz;
    }
  }
  return result;
}
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(kinect_ros2::KinectRosComponent)
