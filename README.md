# `kinect_ros2`

## Interface

### Overview
Basic Kinect-v1 (for the Xbox 360) node, with IPC support, based on [libfreenect](https://github.com/OpenKinect/libfreenect).
For now, it only supports a single Kinect device. (If multiple devices present, the first one listed by the `freenect_num_devices` will be selected).

### Published topics
* `~image_raw` - RGB image(rgb8) ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))
* `~camera_info` - RGB camera_info ([sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html))
* `~depth/image_raw` - Depth image (16UC1, millimetres) ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)); registered to the RGB image by default (see `depth_registered`)
* `~depth/camera_info` - Depth camera_info ([sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html)); the RGB camera's intrinsics and frame when registered
* `~tilt_state` - measured tilt motor angle in degrees, 2 Hz ([std_msgs/Float64](http://docs.ros.org/api/std_msgs/html/msg/Float64.html))

An RGB frame and its depth frame carry the same `header.stamp`. With `image_transport_plugins`
installed, `~image_raw/compressed` (JPEG) and `~depth/image_raw/compressedDepth` (PNG) are
published as well.

### Parameters
* `depth_registered` (bool, default `true`, read-only) - depth aligned to the RGB image (`FREENECT_DEPTH_REGISTERED`), published in frame `kinect_rgb` with the RGB intrinsics; `false` publishes raw depth (`FREENECT_DEPTH_MM`) in `kinect_depth`
* `publish_rate_hz` (double, default `5.0`) - frame pairs published per second; the Kinect's 30 Hz is far too much for a WiFi link; `0` publishes every frame
* `tilt_deg` (double, default `0.0`, range -30..30) - the tilt motor's angle above level; set it with `ros2 param set /kinect/kinect_ros2 tilt_deg 27.0`
* `device_wait_s` (double, default `20.0`, read-only) - how long to wait for a Kinect on the USB bus before the node exits (non-zero, so a launch with `respawn` retries)

Only the camera and motor subdevices are opened (no audio), so no firmware upload re-enumerates the USB hub at start.

## Instalation
### 1. Install libfreenect
The package was tested using a manual build from the [libfreenect](https://github.com/OpenKinect/libfreenect) github because the Kinect used, had a firmware version that requires specific build flags.

### 2. Copy the repo
Copy the repo to your workspace source folder.
~~~
cd ~/ws/src
git clone https://github.com/fadlio/kinect_ros2
~~~

### 3. Install any missing ROS packages
Use `rosdep` from the top directory of your workspace to install any missing ROS related dependency.
~~~
cd ~/ws
rosdep install --from-paths src --ignore-src -r -y
~~~

### 4. Build your workspace
From the top directory of your workspace, use `colcon` to build your packages.
~~~
cd ~/ws
colcon build
~~~

## Using this package

## Devices tested
* Kinect Model 1473