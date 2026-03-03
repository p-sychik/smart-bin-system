# Demo 1 + Demo 2 Teleop Flow

These scripts map exactly to the 4-terminal workflow:

1) Raspberry Pi terminal: base drivers
2) Raspberry Pi terminal: camera
3) DICE terminal: marker-seek node
4) DICE terminal: combined teleop keyboard control

## One-time build on DICE (workspace root)

```bash
cd <workspace-root>
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install \
  --base-paths demo1/turtlebot3_combined_teleop demo2/turtlebot3_marker_seek_demo2
source install/setup.bash
```

## Terminal 1 (Raspberry Pi): bringup

```bash
cd <workspace-root>
bash demo1/start_robot_pi.sh
```

Defaults:
- `TURTLEBOT3_MODEL=burger`

## Terminal 2 (Raspberry Pi): camera

```bash
cd <workspace-root>
bash demo1/start_camera_pi.sh
```

Defaults:
- `CAMERA_DEVICE=/dev/video1`
- `IMAGE_SIZE=[320,240]`
- `PIXEL_FORMAT=YUYV`
- `OUTPUT_ENCODING=mono8`
- `TIME_PER_FRAME=[1,30]`

Override example:

```bash
CAMERA_DEVICE=/dev/video0 IMAGE_SIZE=[640,480] bash demo1/start_camera_pi.sh
```

## Terminal 3 (DICE): marker-seek background behavior

```bash
cd <workspace-root>
bash demo1/start_marker_seek_dice.sh
```

Before running teleop, verify services are visible in another DICE terminal:

```bash
cd <workspace-root>
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=59
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 service list | grep marker_seek
```

## Terminal 4 (DICE): teleop controls

```bash
cd <workspace-root>
bash demo1/start_combined_teleop_dice.sh
```

This runs the same node as:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run turtlebot3_combined_teleop combined_teleop_node
```

## Optional UI Teleop App (DICE with desktop session)

```bash
cd <workspace-root>
bash demo1/start_combined_teleop_ui_dice.sh
```

This runs:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run turtlebot3_combined_teleop combined_teleop_ui_node
```

Keyboard shortcuts (`W/A/S/D`, `C`, `V`, `O`, `P`, `SPACE`, `F`, `T`, `X`, `Q`)
work in the UI window in addition to button clicks.
