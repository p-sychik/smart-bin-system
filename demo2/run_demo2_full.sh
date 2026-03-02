#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

REMOTE_HOST="${REMOTE_HOST:-SNOVER}"
REMOTE_USER="${REMOTE_USER:-ubuntu}"
REMOTE_LOGIN="${REMOTE_USER}@${REMOTE_HOST}"
REMOTE_LOG_DIR="${REMOTE_LOG_DIR:-\$HOME/demo2_logs}"
REMOTE_MODEL="${REMOTE_MODEL:-waffle_pi}"
CAMERA_DEVICE="${CAMERA_DEVICE:-/dev/video0}"
CAMERA_SIZE="${CAMERA_SIZE:-[320,240]}"
CAMERA_PIXEL_FORMAT="${CAMERA_PIXEL_FORMAT:-YUYV}"
CAMERA_ENCODING="${CAMERA_ENCODING:-mono8}"

MARKER_SCRIPT="${SCRIPT_DIR}/run_marker_seek.sh"

usage() {
  cat <<'EOF'
Usage:
  ./demo2/run_demo2_full.sh remote-up
  ./demo2/run_demo2_full.sh remote-down
  ./demo2/run_demo2_full.sh remote-status
  ./demo2/run_demo2_full.sh run [--build]
  ./demo2/run_demo2_full.sh up [--build]

Commands:
  remote-up      SSH to robot and start bringup + camera in background.
  remote-down    Stop bringup + camera started by this script.
  remote-status  Show whether remote bringup/camera processes are running.
  run            Run local Demo 2 marker-seek launcher.
  up             Start remote stack, then run local marker-seek launcher.

Environment overrides:
  REMOTE_HOST (default: SNOVER)
  REMOTE_USER (default: ubuntu)
  REMOTE_MODEL (default: waffle_pi)
  CAMERA_DEVICE (default: /dev/video0)
EOF
}

remote_exec() {
  ssh "${REMOTE_LOGIN}" "$@"
}

remote_exec_script() {
  local script="$1"
  ssh "${REMOTE_LOGIN}" "bash -seu" <<< "${script}"
}

remote_up() {
  local remote_script
  remote_script=$(cat <<EOF
set -euo pipefail
mkdir -p ${REMOTE_LOG_DIR}

if [[ -f ${REMOTE_LOG_DIR}/bringup.pid ]]; then
  old_pid=\$(cat ${REMOTE_LOG_DIR}/bringup.pid || true)
  if [[ -n "\${old_pid}" ]]; then kill "\${old_pid}" 2>/dev/null || true; fi
fi
if [[ -f ${REMOTE_LOG_DIR}/camera.pid ]]; then
  old_pid=\$(cat ${REMOTE_LOG_DIR}/camera.pid || true)
  if [[ -n "\${old_pid}" ]]; then kill "\${old_pid}" 2>/dev/null || true; fi
fi

nohup bash -lc 'source /opt/ros/jazzy/setup.bash; source ~/turtlebot3_ws/install/setup.bash; export TURTLEBOT3_MODEL=${REMOTE_MODEL}; ros2 launch turtlebot3_bringup robot.launch.py' > ${REMOTE_LOG_DIR}/bringup.log 2>&1 &
echo \$! > ${REMOTE_LOG_DIR}/bringup.pid

nohup bash -lc "source /opt/ros/jazzy/setup.bash; source ~/turtlebot3_ws/install/setup.bash; export TURTLEBOT3_MODEL=${REMOTE_MODEL}; ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=${CAMERA_DEVICE} -p image_size:='${CAMERA_SIZE}' -p pixel_format:=${CAMERA_PIXEL_FORMAT} -p output_encoding:=${CAMERA_ENCODING} -r image_raw:=/camera/image_raw" > ${REMOTE_LOG_DIR}/camera.log 2>&1 &
echo \$! > ${REMOTE_LOG_DIR}/camera.pid

sleep 1
echo "bringup_pid=\$(cat ${REMOTE_LOG_DIR}/bringup.pid)"
echo "camera_pid=\$(cat ${REMOTE_LOG_DIR}/camera.pid)"
EOF
)
  remote_exec_script "${remote_script}"
}

remote_down() {
  local remote_script
  remote_script=$(cat <<EOF
set -euo pipefail
for name in bringup camera; do
  pid_file=${REMOTE_LOG_DIR}/\${name}.pid
  if [[ -f "\${pid_file}" ]]; then
    pid=\$(cat "\${pid_file}" || true)
    if [[ -n "\${pid}" ]]; then
      kill "\${pid}" 2>/dev/null || true
      sleep 0.3
      kill -9 "\${pid}" 2>/dev/null || true
    fi
    rm -f "\${pid_file}"
  fi
done
echo "remote processes requested to stop"
EOF
)
  remote_exec_script "${remote_script}"
}

remote_status() {
  local remote_script
  remote_script=$(cat <<EOF
set -euo pipefail
echo "Robot host: \$(hostname)"
for name in bringup camera; do
  pid_file=${REMOTE_LOG_DIR}/\${name}.pid
  if [[ -f "\${pid_file}" ]]; then
    pid=\$(cat "\${pid_file}" || true)
    if [[ -n "\${pid}" ]] && kill -0 "\${pid}" 2>/dev/null; then
      echo "\${name}: running (pid=\${pid})"
    else
      echo "\${name}: not running (stale pid file)"
    fi
  else
    echo "\${name}: not running"
  fi
done
EOF
)
  remote_exec_script "${remote_script}"
}

run_local() {
  "${MARKER_SCRIPT}" run "$@"
}

action="${1:-up}"
shift || true

case "${action}" in
  remote-up)
    remote_up
    ;;
  remote-down)
    remote_down
    ;;
  remote-status)
    remote_status
    ;;
  run)
    run_local "$@"
    ;;
  up)
    remote_up
    run_local "$@"
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
