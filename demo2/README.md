# Demo 2: Marker Seek (ArUco) for TurtleBot3

This folder contains the marker-seek package used for Demo 2.

## What this code does

`marker_seek_node` runs an autonomous marker approach behavior:
1. Spins to search for an ArUco marker.
2. When a marker is detected, estimates marker pose from camera data.
3. Drives toward the marker while steering to center it.
4. Stops at the configured target distance.
5. If marker is lost, retries according to configured limits.

Core behavior logic is in:
- `demo2/turtlebot3_marker_seek_demo2/turtlebot3_marker_seek_demo2/marker_seek_core.py`

ROS interfaces and camera processing are in:
- `demo2/turtlebot3_marker_seek_demo2/turtlebot3_marker_seek_demo2/marker_seek_node.py`

Launch file:
- `demo2/turtlebot3_marker_seek_demo2/launch/marker_seek.launch.py`

## Prerequisites

- ROS 2 Jazzy installed on robot and operator machine.
- TurtleBot3 bringup working (`turtlebot3_bringup`).
- USB camera publishing image and camera info topics.
- `cv_bridge` and OpenCV with `aruco` support available.

## Build (workspace root)

```bash
cd <workspace-root>
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select turtlebot3_marker_seek_demo2
source install/setup.bash
```

## Run

Use separate terminals.

### 1) On TurtleBot (robot-side base + camera)

```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py
```

Then start camera (example for `/dev/video1`):

```bash
source /opt/ros/jazzy/setup.bash
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:=/dev/video1 \
  -p image_size:="[320,240]" \
  -p pixel_format:=YUYV \
  -p output_encoding:=mono8 \
  -r image_raw:=/camera/image_raw
```

### 2) On operator machine (run marker seek)

```bash
cd <workspace-root>
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch turtlebot3_marker_seek_demo2 marker_seek.launch.py
```

Start behavior:

```bash
ros2 service call /marker_seek/start std_srvs/srv/Trigger "{}"
```

Stop behavior:

```bash
ros2 service call /marker_seek/stop std_srvs/srv/Trigger "{}"
```

## Useful checks

```bash
ros2 topic list
ros2 topic hz /camera/image_raw
ros2 topic info /cmd_vel
ros2 service list | grep marker_seek
```

## Key parameters (defaults)

- `image_topic`: `/camera/image_raw`
- `camera_info_topic`: `/camera_info`
- `cmd_vel_topic`: `cmd_vel`
- `aruco_dictionary`: `DICT_4X4_50`
- `marker_size_m`: `0.10`
- `stop_distance_m`: `0.25`
- `search_timeout_s`: `12.0`
- `max_retries`: `3`

Override example (run node directly):

```bash
ros2 run turtlebot3_marker_seek_demo2 marker_seek_node --ros-args \
  -p cmd_vel_topic:=/cmd_vel \
  -p marker_size_m:=0.12
```
