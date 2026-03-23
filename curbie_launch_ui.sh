#!/bin/bash

# ===== CONFIG =====
WORKSPACE="${WORKSPACE:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
SESSION="marker_system"

cd "$WORKSPACE" || exit 1

# shellcheck disable=SC1090
if [[ -f "$HOME/.bashrc" ]]; then
  source "$HOME/.bashrc"
fi

source install/setup.bash

# Kill old tmux session if it exists
tmux kill-session -t $SESSION 2>/dev/null

# Create a new tmux session
tmux new-session -d -s $SESSION

# --- Pane 1: Marker Seek Launch ---
tmux send-keys -t $SESSION "
cd $WORKSPACE
source /opt/ros/jazzy/setup.bash
source ~/.bashrc
colcon build --packages-select turtlebot3_marker_seek_demo2
source install/setup.bash
ros2 launch turtlebot3_marker_seek_demo2 marker_seek.launch.py
" C-m

# --- Pane 2: Combined Teleop UI ---
tmux split-window -h -t $SESSION
tmux send-keys -t $SESSION "
cd $WORKSPACE
source /opt/ros/jazzy/setup.bash
source ~/.bashrc
colcon build --packages-select turtlebot3_combined_teleop
source install/setup.bash
ros2 run turtlebot3_combined_teleop combined_teleop_ui_node
" C-m

# --- Pane 3: Camera View ---
tmux split-window -v -t $SESSION:0.1
tmux send-keys -t $SESSION "
source /opt/ros/jazzy/setup.bash
source ~/.bashrc
ros2 run rqt_image_view rqt_image_view /camera/image_raw
" C-m

# Arrange panes nicely
tmux select-layout -t $SESSION tiled

# Attach to tmux session so you can see everything
tmux attach -t $SESSION
