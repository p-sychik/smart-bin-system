#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

REMOTE_HOST="${REMOTE_HOST:-SNOVER}"
REMOTE_USER="${REMOTE_USER:-ubuntu}"
REMOTE_LOGIN="${REMOTE_USER}@${REMOTE_HOST}"
REMOTE_LOG_DIR="${REMOTE_LOG_DIR:-\$HOME/demo2_logs}"
REMOTE_MODEL="${REMOTE_MODEL:-waffle_pi}"
# SNOVER currently exposes the original front USB camera on this stable by-id path.
FRONT_CAMERA_DEVICE="${FRONT_CAMERA_DEVICE:-${CAMERA_DEVICE:-/dev/v4l/by-id/usb-046d_0825_B10563F0-video-index0}}"
FRONT_CAMERA_SIZE="${FRONT_CAMERA_SIZE:-${CAMERA_SIZE:-[320,240]}}"
FRONT_CAMERA_PIXEL_FORMAT="${FRONT_CAMERA_PIXEL_FORMAT:-${CAMERA_PIXEL_FORMAT:-YUYV}}"
FRONT_CAMERA_ENCODING="${FRONT_CAMERA_ENCODING:-${CAMERA_ENCODING:-mono8}}"
# The newly added rear camera on SNOVER is the Logitech C270 on USB port 1.3.
REAR_CAMERA_DEVICE="${REAR_CAMERA_DEVICE:-/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_200901010001-video-index0}"
REAR_CAMERA_SIZE="${REAR_CAMERA_SIZE:-${FRONT_CAMERA_SIZE}}"
REAR_CAMERA_PIXEL_FORMAT="${REAR_CAMERA_PIXEL_FORMAT:-${FRONT_CAMERA_PIXEL_FORMAT}}"
REAR_CAMERA_ENCODING="${REAR_CAMERA_ENCODING:-${FRONT_CAMERA_ENCODING}}"

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
  remote-up      SSH to robot and start bringup + front/rear cameras in background.
  remote-down    Stop bringup + cameras started by this script.
  remote-status  Show whether remote bringup/camera processes are running.
  run            Run local Demo 2 marker-seek launcher.
  up             Start remote stack, then run local marker-seek launcher.

Environment overrides:
  REMOTE_HOST (default: SNOVER)
  REMOTE_USER (default: ubuntu)
  REMOTE_MODEL (default: waffle_pi)
  FRONT_CAMERA_DEVICE (default: front Logitech UVC by-id path)
  REAR_CAMERA_DEVICE (default: rear Logitech C270 by-id path)
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
for name in front_camera rear_camera; do
  if [[ -f ${REMOTE_LOG_DIR}/\${name}.pid ]]; then
    old_pid=\$(cat ${REMOTE_LOG_DIR}/\${name}.pid || true)
    if [[ -n "\${old_pid}" ]]; then kill "\${old_pid}" 2>/dev/null || true; fi
  fi
done

nohup bash -lc 'source /opt/ros/jazzy/setup.bash; source ~/turtlebot3_ws/install/setup.bash; export TURTLEBOT3_MODEL=${REMOTE_MODEL}; ros2 launch turtlebot3_bringup robot.launch.py' > ${REMOTE_LOG_DIR}/bringup.log 2>&1 &
echo \$! > ${REMOTE_LOG_DIR}/bringup.pid

nohup bash -lc "source /opt/ros/jazzy/setup.bash; source ~/turtlebot3_ws/install/setup.bash; export TURTLEBOT3_MODEL=${REMOTE_MODEL}; ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=${FRONT_CAMERA_DEVICE} -p image_size:='${FRONT_CAMERA_SIZE}' -p pixel_format:=${FRONT_CAMERA_PIXEL_FORMAT} -p output_encoding:=${FRONT_CAMERA_ENCODING} -r image_raw:=/camera/image_raw -r camera_info:=/camera_info" > ${REMOTE_LOG_DIR}/front_camera.log 2>&1 &
echo \$! > ${REMOTE_LOG_DIR}/front_camera.pid

if [[ -n "${REAR_CAMERA_DEVICE}" ]]; then
  nohup bash -lc "source /opt/ros/jazzy/setup.bash; source ~/turtlebot3_ws/install/setup.bash; export TURTLEBOT3_MODEL=${REMOTE_MODEL}; ros2 run v4l2_camera v4l2_camera_node --ros-args -r __node:=rear_v4l2_camera -p video_device:=${REAR_CAMERA_DEVICE} -p image_size:='${REAR_CAMERA_SIZE}' -p pixel_format:=${REAR_CAMERA_PIXEL_FORMAT} -p output_encoding:=${REAR_CAMERA_ENCODING} -r image_raw:=/rear_camera/image_raw -r camera_info:=/rear_camera/camera_info" > ${REMOTE_LOG_DIR}/rear_camera.log 2>&1 &
  echo \$! > ${REMOTE_LOG_DIR}/rear_camera.pid
else
  rm -f ${REMOTE_LOG_DIR}/rear_camera.pid
fi

sleep 1
echo "bringup_pid=\$(cat ${REMOTE_LOG_DIR}/bringup.pid)"
echo "front_camera_pid=\$(cat ${REMOTE_LOG_DIR}/front_camera.pid)"
if [[ -f ${REMOTE_LOG_DIR}/rear_camera.pid ]]; then
  echo "rear_camera_pid=\$(cat ${REMOTE_LOG_DIR}/rear_camera.pid)"
else
  echo "rear_camera_pid=disabled"
fi
EOF
)
  remote_exec_script "${remote_script}"
}

remote_down() {
  local remote_script
  remote_script=$(cat <<EOF
set -euo pipefail
for name in bringup front_camera rear_camera; do
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
for name in bringup front_camera rear_camera; do
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
