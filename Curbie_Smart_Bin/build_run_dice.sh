#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/dice_ws"

set +u
source /opt/ros/jazzy/setup.bash
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-59}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="SUBNET"

DICE_HOST="$(hostname -s)"
export CYCLONEDDS_URI="<Discovery><ParticipantIndex>auto</><Peers><Peer Address='snover.inf.ed.ac.uk' /><Peer Address='${DICE_HOST}.inf.ed.ac.uk' /></></><Tracing><Verbosity>config</><Out>stderr</></><General><AllowMulticast>false</></>"

colcon build --base-paths src --symlink-install
set +u
source "$SCRIPT_DIR/dice_ws/install/setup.bash"
set -u

exec ros2 launch curbie_smart_bin_operator curbie_smart_bin_app.launch.py
