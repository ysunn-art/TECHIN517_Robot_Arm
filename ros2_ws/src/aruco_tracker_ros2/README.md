# aruco_tracker_ros2

A ROS2 package for detecting ArUco markers and publishing their 6D poses as TF transforms. It provides two nodes:

- **ArucoCubeTracker** -- tracks a cube with ArUco markers on each face (IDs 0--5) and publishes the cube center as a TF frame.
- **ArucoCodeTracker** -- tracks individual ArUco markers and publishes each marker's pose as a TF frame.

Both nodes support multiple cameras, optional debug image visualization, and configurable ArUco dictionaries. Image conversion is handled internally (no `cv_bridge` dependency).

## Installation

### Dependencies

**ROS2 packages:**

- `rclpy`
- `sensor_msgs`
- `geometry_msgs`
- `tf2_ros`

**Python libraries:**

- `opencv-python` (with the `aruco` contrib module)
- `scipy`
- `numpy`

### Build

```bash
# From your workspace root
cd ~/your_ws/src
git clone https://github.com/GIXLabs/aruco_tracker_ros2.git aruco_tracker_ros2

cd ~/your_ws
rosdep install --from-paths src --ignore-src -y
colcon build --packages-select aruco_tracker_ros2
source install/setup.bash
```

## Configuration

Parameter files live in the `config/` directory. Edit these to match your camera setup.

### Common parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `image_topics` | string[] | `["/camera/image_raw"]` | Image topic for each camera |
| `camera_info_topics` | string[] | `["/camera/camera_info"]` | Camera calibration topic for each camera |
| `camera_names` | string[] | `["camera"]` | Human-readable name for each camera |
| `marker_size` | double | `0.05` | Physical marker side length in meters |
| `aruco_dictionary` | int | `0` | OpenCV ArUco dictionary ID (0 = `DICT_4X4_50`) |
| `publish_debug_images` | bool | `true` | Publish annotated debug images |

All three array parameters must have the same length.

### Cube tracker parameters (`config/aruco_cube_tracker.yaml`)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `cube_size` | double | `0.03` | Physical cube side length in meters |
| `cube_frame_id` | string | `"aruco_cube"` | TF child frame name for the cube center |

### Code tracker parameters (`config/aruco_code_tracker.yaml`)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `marker_id` | int | `-1` | Marker ID to track (`-1` = track all detected markers) |
| `marker_frame_id` | string | `"aruco_code"` | TF frame name; used as a prefix (`{marker_frame_id}_{id}`) when tracking all markers |

## Usage

### Launch files

```bash
# Cube tracker with default config
ros2 launch aruco_tracker_ros2 aruco_cube_tracker.launch.py

# Cube tracker with a custom frame name
ros2 launch aruco_tracker_ros2 aruco_cube_tracker.launch.py cube_frame_id:=my_cube

# Code tracker -- all markers
ros2 launch aruco_tracker_ros2 aruco_code_tracker.launch.py

# Code tracker -- single marker
ros2 launch aruco_tracker_ros2 aruco_code_tracker.launch.py marker_id:=5

# Either tracker with a custom config file
ros2 launch aruco_tracker_ros2 aruco_cube_tracker.launch.py config_file:=/path/to/custom.yaml
```

### Running nodes directly

```bash
ros2 run aruco_tracker_ros2 aruco_cube_tracker --ros-args --params-file config/aruco_cube_tracker.yaml
ros2 run aruco_tracker_ros2 aruco_code_tracker --ros-args --params-file config/aruco_code_tracker.yaml
```

### Topics

| Topic | Type | Description |
|-------|------|-------------|
| `<image_topics>` | `sensor_msgs/Image` | Subscribed camera image streams |
| `<camera_info_topics>` | `sensor_msgs/CameraInfo` | Subscribed camera calibration data |
| `/tf` | `geometry_msgs/TransformStamped` | Published marker/cube pose transforms |
| `~/debug/<camera_name>` | `sensor_msgs/Image` | Published debug images with marker overlays (when enabled) |

### TF frames

- **Cube tracker**: publishes `cube_frame_id` (default `aruco_cube`) relative to the camera frame.
- **Code tracker**: publishes `marker_frame_id` for a single marker, or `{marker_frame_id}_{id}` for each marker when tracking all.

## Acknowledgements

- [OpenCV ArUco module](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html) for marker detection and pose estimation.
- [ROS2](https://docs.ros.org/) and [tf2](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Tf2.html) for the robotics middleware and transform framework.

## License

This project is licensed under the Apache License 2.0. See the [LICENSE](LICENSE) file for details.
