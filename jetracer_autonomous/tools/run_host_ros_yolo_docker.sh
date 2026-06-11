#!/usr/bin/env bash
set -euo pipefail

CATKIN_WS="${CATKIN_WS:-$HOME/catkin_ws}"
REPO_PATH="${REPO_PATH:-$CATKIN_WS/src/jetracer}"
ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-melodic}"
YOLO_IMAGE="${YOLO_IMAGE:-ultralytics/ultralytics:latest-jetson-jetpack4}"
YOLO_CONTAINER="${YOLO_CONTAINER:-jetracer_yolo_http}"
YOLO_PORT="${YOLO_PORT:-8765}"
YOLO_CONF="${YOLO_CONF:-0.6}"
YOLO_DEVICE="${YOLO_DEVICE:-0}"
YOLO_IMGSZ="${YOLO_IMGSZ:-640}"
YOLO_HALF="${YOLO_HALF:-0}"
CAMERA_WIDTH="${CAMERA_WIDTH:-640}"
CAMERA_HEIGHT="${CAMERA_HEIGHT:-480}"
CAMERA_FPS="${CAMERA_FPS:-15}"
CAMERA_CAPTURE_WIDTH="${CAMERA_CAPTURE_WIDTH:-1280}"
CAMERA_CAPTURE_HEIGHT="${CAMERA_CAPTURE_HEIGHT:-720}"
CAMERA_CAPTURE_FPS="${CAMERA_CAPTURE_FPS:-30}"
MODEL_PATH="${MODEL_PATH:-$REPO_PATH/jetracer_autonomous/models/best.pt}"
CONFIG_PATH="${CONFIG_PATH:-$REPO_PATH/jetracer_autonomous/config/params.yaml}"
DOCKER_BIN="${DOCKER_BIN:-sudo docker}"
SUDO_BIN="${SUDO_BIN:-sudo}"
STOP_HARDWARE_PROCESSES="${STOP_HARDWARE_PROCESSES:-1}"
KILL_VIDEO_DEVICE_USERS="${KILL_VIDEO_DEVICE_USERS:-1}"
RESTART_NVARGUS="${RESTART_NVARGUS:-1}"
CAMERA_DEVICE="${CAMERA_DEVICE:-/dev/video0}"

ROSCORE_PID=""
CAMERA_PID=""
STARTED_ROSCORE=0

cleanup() {
  echo
  echo "[run_stack] stopping..."
  if [[ -n "${CAMERA_PID}" ]] && kill -0 "${CAMERA_PID}" 2>/dev/null; then
    kill "${CAMERA_PID}" 2>/dev/null || true
  fi
  ${DOCKER_BIN} rm -f "${YOLO_CONTAINER}" >/dev/null 2>&1 || true
  if [[ "${STARTED_ROSCORE}" == "1" && -n "${ROSCORE_PID}" ]] && kill -0 "${ROSCORE_PID}" 2>/dev/null; then
    kill "${ROSCORE_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

require_file() {
  if [[ ! -e "$1" ]]; then
    echo "[run_stack] missing: $1" >&2
    exit 1
  fi
}

wait_for_roscore() {
  for _ in $(seq 1 20); do
    if rostopic list >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.5
  done
  echo "[run_stack] roscore did not become ready" >&2
  exit 1
}

wait_for_http() {
  local url="http://127.0.0.1:${YOLO_PORT}/health"
  for _ in $(seq 1 60); do
    if command -v curl >/dev/null 2>&1 && curl -fsS "${url}" >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.5
  done
  echo "[run_stack] YOLO HTTP service did not become ready at ${url}" >&2
  exit 1
}

wait_for_camera() {
  local topic="/camera/image_raw"
  for _ in $(seq 1 30); do
    if [[ -n "${CAMERA_PID}" ]] && ! kill -0 "${CAMERA_PID}" 2>/dev/null; then
      echo "[run_stack] camera node exited while waiting for ${topic}. Last log lines:" >&2
      tail -60 /tmp/jetracer_csi_camera.log >&2 || true
      exit 1
    fi

    if rostopic list 2>/dev/null | grep -qx "${topic}"; then
      if timeout 2 rostopic echo "${topic}/header" -n 1 >/dev/null 2>&1; then
        return 0
      fi
    fi

    sleep 0.5
  done

  echo "[run_stack] camera did not publish frames on ${topic}. Last log lines:" >&2
  tail -80 /tmp/jetracer_csi_camera.log >&2 || true
  echo "[run_stack] active publishers:" >&2
  rostopic info "${topic}" >&2 || true
  exit 1
}

kill_ros_node_if_present() {
  local node="$1"
  if rosnode list 2>/dev/null | grep -qx "${node}"; then
    echo "[run_stack] stopping ROS node ${node}"
    rosnode kill "${node}" >/dev/null 2>&1 || true
    sleep 0.5
  fi
}

stop_hardware_processes() {
  if [[ "${STOP_HARDWARE_PROCESSES}" != "1" && "${STOP_HARDWARE_PROCESSES}" != "true" ]]; then
    return 0
  fi

  echo "[run_stack] stopping old hardware/process owners"
  ${DOCKER_BIN} rm -f "${YOLO_CONTAINER}" >/dev/null 2>&1 || true

  if rostopic list >/dev/null 2>&1; then
    kill_ros_node_if_present "/jetracer_autonomous_drive"
    kill_ros_node_if_present "/csi_camera_node"
    kill_ros_node_if_present "/gscam"
    kill_ros_node_if_present "/usb_cam"
    kill_ros_node_if_present "/usb_cam_node"
  fi

  if [[ "${KILL_VIDEO_DEVICE_USERS}" == "1" || "${KILL_VIDEO_DEVICE_USERS}" == "true" ]]; then
    if [[ -e "${CAMERA_DEVICE}" ]] && command -v fuser >/dev/null 2>&1; then
      echo "[run_stack] releasing ${CAMERA_DEVICE}"
      ${SUDO_BIN} fuser -k "${CAMERA_DEVICE}" >/dev/null 2>&1 || true
      sleep 1
    fi
  fi

  if [[ "${RESTART_NVARGUS}" == "1" || "${RESTART_NVARGUS}" == "true" ]]; then
    if command -v systemctl >/dev/null 2>&1; then
      echo "[run_stack] restarting nvargus-daemon"
      ${SUDO_BIN} systemctl restart nvargus-daemon >/dev/null 2>&1 || true
      sleep 2
    fi
  fi
}

require_file "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
require_file "${CATKIN_WS}/devel/setup.bash"
require_file "${REPO_PATH}/jetracer_autonomous/tools/yolo_http_service.py"
require_file "${MODEL_PATH}"
require_file "${CONFIG_PATH}"

source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
source "${CATKIN_WS}/devel/setup.bash"

echo "[run_stack] repo: ${REPO_PATH}"
echo "[run_stack] config: ${CONFIG_PATH}"
echo "[run_stack] model: ${MODEL_PATH}"
echo "[run_stack] yolo device: ${YOLO_DEVICE}"
echo "[run_stack] camera: ${CAMERA_WIDTH}x${CAMERA_HEIGHT}@${CAMERA_FPS} capture=${CAMERA_CAPTURE_WIDTH}x${CAMERA_CAPTURE_HEIGHT}@${CAMERA_CAPTURE_FPS}"
echo "[run_stack] cleanup hardware: ${STOP_HARDWARE_PROCESSES} kill_device_users=${KILL_VIDEO_DEVICE_USERS} restart_nvargus=${RESTART_NVARGUS}"

if ! rostopic list >/dev/null 2>&1; then
  echo "[run_stack] starting roscore"
  roscore >/tmp/jetracer_roscore.log 2>&1 &
  ROSCORE_PID="$!"
  STARTED_ROSCORE=1
  wait_for_roscore
else
  echo "[run_stack] roscore already running"
fi

stop_hardware_processes

echo "[run_stack] starting CSI camera node"
rosrun jetracer_autonomous csi_camera_node.py \
  _width:="${CAMERA_WIDTH}" \
  _height:="${CAMERA_HEIGHT}" \
  _fps:="${CAMERA_FPS}" \
  _capture_width:="${CAMERA_CAPTURE_WIDTH}" \
  _capture_height:="${CAMERA_CAPTURE_HEIGHT}" \
  _capture_fps:="${CAMERA_CAPTURE_FPS}" \
  >/tmp/jetracer_csi_camera.log 2>&1 &
CAMERA_PID="$!"
sleep 2
if ! kill -0 "${CAMERA_PID}" 2>/dev/null; then
  echo "[run_stack] camera node exited. Last log lines:" >&2
  tail -40 /tmp/jetracer_csi_camera.log >&2 || true
  exit 1
fi

echo "[run_stack] waiting for camera frames"
wait_for_camera
echo "[run_stack] camera is publishing frames"

echo "[run_stack] starting YOLO HTTP Docker container"
${DOCKER_BIN} rm -f "${YOLO_CONTAINER}" >/dev/null 2>&1 || true
YOLO_HALF_FLAG=""
if [[ "${YOLO_HALF}" == "1" || "${YOLO_HALF}" == "true" ]]; then
  YOLO_HALF_FLAG="--half"
fi
${DOCKER_BIN} run -d --rm \
  --name "${YOLO_CONTAINER}" \
  --network host \
  --runtime nvidia \
  -v "${REPO_PATH}:/workspace/jetracer" \
  "${YOLO_IMAGE}" \
  bash -lc "cd /workspace/jetracer && python3 jetracer_autonomous/tools/yolo_http_service.py --model /workspace/jetracer/jetracer_autonomous/models/best.pt --host 0.0.0.0 --port ${YOLO_PORT} --conf ${YOLO_CONF} --device ${YOLO_DEVICE} --imgsz ${YOLO_IMGSZ} ${YOLO_HALF_FLAG}" >/tmp/jetracer_yolo_container_id.txt

wait_for_http
echo "[run_stack] YOLO HTTP service is ready"

echo "[run_stack] launching autonomous node"
roslaunch jetracer_autonomous autonomous_drive.launch config_path:="${CONFIG_PATH}"
