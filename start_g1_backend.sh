#!/bin/bash
# Run on the G1 body. Starts the local DDS/RPC control backend and web UI.

set -e

BASE="$(cd "$(dirname "$0")" && pwd)"
RUNTIME_DIR="${BASE}/.runtime"
mkdir -p "$RUNTIME_DIR"

export HONGTU_G1_NET_IF="${HONGTU_G1_NET_IF:-auto}"
export HONGTU_SERVICE_HOST="${HONGTU_SERVICE_HOST:-0.0.0.0}"
export HONGTU_SERVICE_PORT="${HONGTU_SERVICE_PORT:-5055}"
export CYCLONEDDS_HOME="${CYCLONEDDS_HOME:-/home/unitree/cyclonedds_ws/install/cyclonedds}"
export PYTHONPATH="${RUNTIME_DIR}/python:${BASE}/unitree_sdk2_python:${BASE}/inspire_hand:${PYTHONPATH}"
export LD_LIBRARY_PATH="${CYCLONEDDS_HOME}/lib:/usr/local/athena/third_party/arm64/lib/unitree_sdk2:${LD_LIBRARY_PATH}"
export LANG="${LANG:-en_US.UTF-8}"
export LC_CTYPE="${LC_CTYPE:-${LANG}}"

if [ -f /opt/ros/noetic/setup.bash ]; then
    source /opt/ros/noetic/setup.bash
fi

cd "$BASE"

PID_R=""
PID_L=""
if python3 - <<'PY'
import pymodbus
PY
then
    echo "[backend] 启动右手驱动 (192.168.123.211)…"
    python3 inspire_hand_driver.py --lr r --tcp-ip 192.168.123.211 --network "$HONGTU_G1_NET_IF" &
    PID_R=$!

    echo "[backend] 启动左手驱动 (192.168.123.210)…"
    python3 inspire_hand_driver.py --lr l --tcp-ip 192.168.123.210 --network "$HONGTU_G1_NET_IF" &
    PID_L=$!
else
    echo "[backend] 跳过灵巧手 Modbus 驱动：未安装 pymodbus"
fi

cleanup() {
    [ -n "$PID_R" ] && kill "$PID_R" 2>/dev/null || true
    [ -n "$PID_L" ] && kill "$PID_L" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "[backend] HongTu: $BASE"
echo "[backend] DDS net_if: $HONGTU_G1_NET_IF"
echo "[backend] HTTP: http://0.0.0.0:${HONGTU_SERVICE_PORT}"
python3 g1_robot_service.py --host "$HONGTU_SERVICE_HOST" --port "$HONGTU_SERVICE_PORT" --net-if "$HONGTU_G1_NET_IF"
