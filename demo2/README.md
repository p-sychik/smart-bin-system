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
- `python3-serial` installed on the robot if using direct Pico serial control.

## Build (workspace root)

```bash
cd <workspace-root>
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select turtlebot3_marker_seek_demo2
source install/setup.bash
```

## Run

Use separate terminals.

### Quick runner script

From workspace root:

```bash
./demo2/run_marker_seek.sh build
./demo2/run_marker_seek.sh run
```

Useful controls in another terminal:

```bash
./demo2/run_marker_seek.sh start
./demo2/run_marker_seek.sh stop
./demo2/run_marker_seek.sh status
```

### Full end-to-end helper (robot + operator)

This script SSHes to the robot (default `ubuntu@SNOVER`) to start bringup and
camera in the background, then runs Demo 2 locally:

```bash
./demo2/run_demo2_full.sh up --build
```

Useful remote controls:

```bash
./demo2/run_demo2_full.sh remote-status
./demo2/run_demo2_full.sh remote-down
```

Override robot host if needed:

```bash
REMOTE_HOST=snover ./demo2/run_demo2_full.sh up
```

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

## Hook + Pico port check

Run these on the TurtleBot before launching marker seek:

```bash
ls -l /dev/ttyACM*
```

Expected setup is usually:
- OpenCR on `/dev/ttyACM0` (base controller)
- Pico on `/dev/ttyACM1` (hook servos)

Quick Pico check:

```bash
python3 - <<'PY'
import serial
ser = serial.Serial('/dev/ttyACM1', 115200, timeout=1.0)
ser.write(b'STATUS\n')
print(ser.readline().decode('utf-8', 'ignore').strip())
ser.close()
PY
```

When marker seek reaches `SUCCEEDED`, it now auto-deploys the hook by:
- publishing lock servo values on `servo_cmd`
- sending `LOCK` to Pico serial (if enabled)

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
- `marker_x_sign`: `-1.0` (flip to `+1.0` if robot steers away from marker)
- `hook_auto_deploy`: `true` (deploy on `SUCCEEDED`)
- `hook_servo_topic`: `servo_cmd`
- `hook_deploy_left`: `0.7`
- `hook_deploy_right`: `0.0` (single-servo setup)
- `pico_serial_enable`: `true` in launch
- `pico_serial_port`: `/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e663682593756333-if00`

Override example (run node directly):

```bash
ros2 run turtlebot3_marker_seek_demo2 marker_seek_node --ros-args \
  -p cmd_vel_topic:=/cmd_vel \
  -p marker_size_m:=0.12
```
