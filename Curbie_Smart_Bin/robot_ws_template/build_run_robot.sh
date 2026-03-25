#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

set +u
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash
set -u

# Keep launch env explicit so non-interactive shells do not miss robot vars.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-59}"
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-waffle_pi}"
export LDS_MODEL="${LDS_MODEL:-LDS-01}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="SUBNET"

if [[ -f "$HOME/DICE.peer" ]]; then
    PEER_HOST="$(grep -oe '^[^\.]*' "$HOME/DICE.peer" | head -n 1 || true)"
    if [[ -n "$PEER_HOST" ]]; then
        export CYCLONEDDS_URI="<Discovery><ParticipantIndex>auto</><Peers><Peer Address='snover.inf.ed.ac.uk' /><Peer Address='${PEER_HOST}.inf.ed.ac.uk' /></></><Tracing><Verbosity>config</><Out>stderr</></><General><AllowMulticast>false</></>"
    fi
fi

colcon build --base-paths src --symlink-install
set +u
source "$SCRIPT_DIR/install/setup.bash"
set -u

exec ros2 launch curbie_smart_bin_robot curbie_smart_bin_robot.launch.py
