#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Avoid unbound-variable failure under `set -u` when sourcing ROS setup.
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES-}"

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
if [[ -f "${WS_ROOT}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${WS_ROOT}/install/setup.bash"
fi

if ! ros2 pkg prefix turtlebot3_combined_teleop >/dev/null 2>&1; then
  echo "Package turtlebot3_combined_teleop not found in this workspace." >&2
  echo "Build first from ${WS_ROOT}:" >&2
  echo "  colcon build --symlink-install --base-paths demo1/turtlebot3_combined_teleop" >&2
  exit 1
fi

exec ros2 run turtlebot3_combined_teleop combined_teleop_ui_node "$@"
