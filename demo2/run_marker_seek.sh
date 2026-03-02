#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

PKG="turtlebot3_marker_seek_demo2"
LAUNCH_FILE="marker_seek.launch.py"

usage() {
  cat <<'EOF'
Usage:
  ./demo2/run_marker_seek.sh build
  ./demo2/run_marker_seek.sh run [--build]
  ./demo2/run_marker_seek.sh start
  ./demo2/run_marker_seek.sh stop
  ./demo2/run_marker_seek.sh status

Commands:
  build   Build Demo 2 package.
  run     Launch marker seek node (foreground). Use --build to build first.
  start   Call /marker_seek/start service.
  stop    Call /marker_seek/stop service.
  status  Show marker-seek node and service visibility.
EOF
}

source_ros() {
  local nounset_was_on=0
  if [[ $- == *u* ]]; then
    nounset_was_on=1
    set +u
  fi
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  if [[ -f "${WS_ROOT}/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "${WS_ROOT}/install/setup.bash"
  fi
  if (( nounset_was_on )); then
    set -u
  fi
}

build_pkg() {
  source_ros
  cd "${WS_ROOT}"
  colcon build \
    --symlink-install \
    --base-paths demo2/turtlebot3_marker_seek_demo2 \
    --packages-select "${PKG}"
}

action="${1:-run}"
shift || true

case "${action}" in
  build)
    build_pkg
    ;;
  run)
    if [[ "${1:-}" == "--build" ]]; then
      build_pkg
      shift || true
    fi
    source_ros
    cd "${WS_ROOT}"
    exec ros2 launch "${PKG}" "${LAUNCH_FILE}" "$@"
    ;;
  start)
    source_ros
    ros2 service call /marker_seek/start std_srvs/srv/Trigger "{}"
    ;;
  stop)
    source_ros
    ros2 service call /marker_seek/stop std_srvs/srv/Trigger "{}"
    ;;
  status)
    source_ros
    echo "Nodes:"
    ros2 node list | grep -E 'marker_seek|^$' || true
    echo
    echo "Services:"
    ros2 service list | grep marker_seek || true
    ;;
  -h|--help|help)
    usage
    ;;
  *)
    echo "Unknown command: ${action}" >&2
    usage
    exit 1
    ;;
esac
