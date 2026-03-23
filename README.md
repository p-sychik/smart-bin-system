# Smart Bin System

This workspace contains the current multi-bin collection stack for the TurtleBot-based smart bin system.

The workflow this README documents is the one currently wired into the codebase:

- the robot starts at home
- it goes `home -> bin 10 -> collection`
- it drops the bin at collection using the rear camera
- it goes `collection -> bin 11 -> collection`
- it drops the second bin
- it finally goes `collection -> home` using a dedicated direct return path

## Current Assumptions

- DICE machine hostname: `kaktusek`
- TurtleBot hostname: `SNOVER`
- pickup bin ArUco markers: `10` and `11`
- collection ArUco marker: `40`
- TurtleBot model on SNOVER: `waffle_pi`

The collection marker ID is configured in [layer4_mission/mission_controller/config/collection_layout.json](layer4_mission/mission_controller/config/collection_layout.json).

## What Runs Where

### On SNOVER

`./curbie_setup.sh` starts:

- TurtleBot base bringup
- servo driver
- front camera on `/camera/image_raw`
- rear camera on `/rear_camera/image_raw`

### On the DICE machine

You need these processes:

- backend API
- path manager
- marker seek / align-to-marker
- hook controller
- mission controller
- teleop + mission setup UI

## Bringup

### 1. Robot-side bringup on SNOVER

```bash
ssh ubuntu@SNOVER
./curbie_setup.sh
```

Expected signs:

- servos initialize
- front camera light turns on
- rear camera light turns on

### 2. DICE-side bringup

From the repo root:

```bash
cd ~/smart-bin-system
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Start the backend:

```bash
cd ~/smart-bin-system/layer5_interface/backend
python3 -m uvicorn main:app --host 127.0.0.1 --port 8000
```

Start the path manager:

```bash
cd ~/smart-bin-system
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run path_manager path_manager_node
```

Start marker seek:

```bash
cd ~/smart-bin-system
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch turtlebot3_marker_seek_demo2 marker_seek.launch.py
```

Start the hook controller:

```bash
cd ~/smart-bin-system
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run hook_controller_pkg hook_controller
```

Start the mission controller:

```bash
cd ~/smart-bin-system
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run mission_controller mission_controller
```

Start the UI:

```bash
cd ~/smart-bin-system
./demo1/start_combined_teleop_ui_dice.sh
```

The UI window title is:

- `TurtleBot3 Teleop + Mission Setup`

## Quick Health Checks

Run these on the DICE machine:

```bash
source ~/.bashrc >/dev/null 2>&1
source /opt/ros/jazzy/setup.bash
source ~/smart-bin-system/install/setup.bash
ros2 service list | grep -E '/path_manager/start_recording|/path_manager/stop_recording|/mission/start|/hook/engage|/marker_seek/start'
```

```bash
source ~/.bashrc >/dev/null 2>&1
source /opt/ros/jazzy/setup.bash
source ~/smart-bin-system/install/setup.bash
ros2 topic list | grep -E '/camera/image_raw|/rear_camera/image_raw|/mission/status|/hook/status'
```

## UI Overview

The UI has three parts that matter for multi-bin testing:

### 1. Driving controls

Use these to position the robot while recording paths:

- `W/A/S/D` or the movement buttons
- `X` to stop
- `C` to disengage the hook
- `V` to engage the hook
- `F` to run marker seek

### 2. Path + Bin Setup

This is where you:

- record paths
- assign the last recorded path to a field
- create bin entries in the backend

Each bin now stores three path IDs:

- `path_to_bin_id`: `home -> bin staging`
- `path_to_collection_id`: `bin staging -> collection staging`
- `path_to_home_id`: `collection staging -> home`

### 3. Mission Queue

This is where you:

- refresh the list of registered bins
- select a bin
- queue it into the mission controller
- watch live mission progress

## Important Recording Rule

For each bin, the controller assumes:

- `path_to_bin_id` starts at home and ends at that bin's pickup staging pose
- `path_to_collection_id` starts at that bin's pickup staging pose and ends at collection staging
- `path_to_home_id` starts at collection staging and ends at home

If both bins share the same collection area and same home pose, you should record `collection -> home` once and use the same `path_to_home_id` for both bin entries.

## Exact Test Flow For Markers 10, 11, and 40

### Physical setup

1. Put the robot in a repeatable home pose.
2. Place bin marker `10` where the front camera can align to it from a staging pose.
3. Place bin marker `11` the same way.
4. Place collection marker `40` where the rear camera can see it from the dropoff staging pose.

### Record the shared collection-to-home path first

This path can be reused by both bins.

1. Manually place the robot at the collection staging pose.
2. In the UI `Path Recorder` section:
   - `Path ID`: `collection40_to_home`
   - `Description`: `collection marker 40 to home`
3. Click `Start Path Recording`.
4. Drive from the collection staging pose back to the exact home pose.
5. Click `Stop Path Recording`.
6. Click `Use Last Path for Path To Home`.

### Register bin 10

1. Put the robot back at the exact home pose.
2. Record `home -> bin 10`:
   - `Path ID`: `home_to_bin10`
   - `Description`: `home to bin 10 staging`
   - Click `Start Path Recording`
   - Drive to a good pickup staging pose in front of marker `10`
   - Click `Stop Path Recording`
   - Click `Use Last Path for Path To Bin`
3. Make sure the hook is open with `C`.
4. Click `Start Seek (F)` and let the robot align to marker `10`.
5. Record `bin 10 -> collection 40`:
   - `Path ID`: `bin10_to_collection40`
   - `Description`: `bin 10 staging to collection 40`
   - Click `Start Path Recording`
   - Drive from the aligned pickup pose to the collection staging pose
   - End with marker `40` behind the robot
   - Click `Stop Path Recording`
   - Click `Use Last Path for Path To Collection`
6. In `Bin Registration`, enter:
   - `RFID Tag ID`: `rfid-10`
   - `Bin Type`: `waste`
   - `Location Name`: `Bin 10`
   - `Path To Bin ID`: `home_to_bin10`
   - `Path To Collection ID`: `bin10_to_collection40`
   - `Path To Home ID`: `collection40_to_home`
   - `Pickup Marker ID`: `10`
7. Click `Create Bin in Backend`.

### Register bin 11

1. Put the robot back at the exact home pose.
2. Record `home -> bin 11`:
   - `Path ID`: `home_to_bin11`
   - `Description`: `home to bin 11 staging`
   - Click `Start Path Recording`
   - Drive to a good pickup staging pose in front of marker `11`
   - Click `Stop Path Recording`
   - Click `Use Last Path for Path To Bin`
3. Make sure the hook is open with `C`.
4. Click `Start Seek (F)` and let the robot align to marker `11`.
5. Record `bin 11 -> collection 40`:
   - `Path ID`: `bin11_to_collection40`
   - `Description`: `bin 11 staging to collection 40`
   - Click `Start Path Recording`
   - Drive to the same collection staging pose used above
   - Click `Stop Path Recording`
   - Click `Use Last Path for Path To Collection`
6. In `Bin Registration`, enter:
   - `RFID Tag ID`: `rfid-11`
   - `Bin Type`: `waste`
   - `Location Name`: `Bin 11`
   - `Path To Bin ID`: `home_to_bin11`
   - `Path To Collection ID`: `bin11_to_collection40`
   - `Path To Home ID`: `collection40_to_home`
   - `Pickup Marker ID`: `11`
7. Click `Create Bin in Backend`.

### Queue the two-bin mission

1. Put the robot back at home.
2. In `Mission Queue`, click `Refresh Bin List`.
3. Select the `Bin 10` entry.
4. Click `Use Highlighted Bin`.
5. Click `Queue Mission`.
6. Select the `Bin 11` entry.
7. Click `Use Highlighted Bin`.
8. Click `Queue Mission`.

## Expected Robot Behavior

With the current controller logic, the robot should do this:

1. `home -> bin 10`
2. align to marker `10` using the front camera
3. hook the bin
4. `bin 10 -> collection 40`
5. align to marker `40` using the rear camera
6. drop the bin into slot 1
7. leave collection
8. go to bin `11`
9. align to marker `11` using the front camera
10. hook the second bin
11. go back to collection `40`
12. align with the rear camera
13. drop the second bin into slot 2
14. execute the direct `collection40_to_home` path

## Expected Mission Status Sequence

In the UI `Live Mission Status` panel, you should see phases like:

- `PATH_TO_BIN`
- `ALIGN_PICKUP`
- `ENGAGE_HOOK`
- `PATH_TO_COLLECTION`
- `ALIGN_DROPOFF`
- `DISENGAGE_HOOK`
- `BACKING_OUT`
- `RETURN_HOME`

For the second queued bin, you should also see:

- `RETURN_TO_BIN_STAGING`

That is the controller using the recorded `bin -> collection` path in reverse to leave collection and get back toward the next bin.

## Re-running The Test

Collection slots are marked occupied once a dropoff succeeds. Before re-running another full test, clear them:

```bash
cd ~/smart-bin-system
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 service call /mission/clear_collection_slots std_srvs/srv/Trigger "{}"
```

You do not need to recreate the bins unless the path IDs or marker IDs changed.

## Troubleshooting

### Rear camera is dark or not used

- confirm SNOVER ran `./curbie_setup.sh`
- confirm `/rear_camera/image_raw` exists on the DICE machine
- for dropoff alignment, the hook must be attached so the system selects the rear camera

### Final return home does not start

This usually means the queued bin entry has no `Path To Home ID`.

Fix:

- record `collection -> home`
- set `Path To Home ID`
- recreate or update the bin entry

### Pickup alignment fails

Check:

- the correct bin marker ID was entered
- the robot ends the recorded `home -> bin` path at a good staging pose
- marker `10` or `11` is visible to the front camera

### Dropoff alignment fails

Check:

- collection marker `40` is actually mounted and visible
- the robot ends the `bin -> collection` path at a rear-camera-friendly pose
- the hook is attached before dropoff alignment

## Code Areas

The main files for this workflow are:

- [demo1/turtlebot3_combined_teleop/turtlebot3_combined_teleop/combined_teleop_ui_node.py](demo1/turtlebot3_combined_teleop/turtlebot3_combined_teleop/combined_teleop_ui_node.py)
- [layer4_mission/mission_controller/mission_controller/mission_controller.py](layer4_mission/mission_controller/mission_controller/mission_controller.py)
- [layer4_mission/mission_controller/config/collection_layout.json](layer4_mission/mission_controller/config/collection_layout.json)
- [layer5_interface/backend/main.py](layer5_interface/backend/main.py)
- [layer5_interface/backend/models.py](layer5_interface/backend/models.py)
- [layer5_interface/backend/schemas.py](layer5_interface/backend/schemas.py)
