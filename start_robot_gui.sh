#!/bin/bash
# Run on the G1 body. PC only needs remote display (SSH -X/VNC/NoMachine).
# This keeps DDS/RPC local to the robot and isolates HongTu runtime config.

set -e

BASE="$(cd "$(dirname "$0")" && pwd)"
RUNTIME_DIR="${BASE}/.runtime"
mkdir -p "$RUNTIME_DIR"

export HONGTU_CONFIG_FILE="${HONGTU_CONFIG_FILE:-${RUNTIME_DIR}/g1_nav_panel.json}"
export HONGTU_G1_NET_IF="${HONGTU_G1_NET_IF:-auto}"
export HONGTU_FORCE_NET_IF="${HONGTU_FORCE_NET_IF:-1}"
export HONGTU_ROBOT_MODE="${HONGTU_ROBOT_MODE:-1}"
export CYCLONEDDS_HOME="${CYCLONEDDS_HOME:-/home/unitree/cyclonedds_ws/install/cyclonedds}"
export PYTHONPATH="${RUNTIME_DIR}/python:${BASE}/unitree_sdk2_python:${BASE}/inspire_hand:${PYTHONPATH}"
export LD_LIBRARY_PATH="${CYCLONEDDS_HOME}/lib:/usr/local/athena/third_party/arm64/lib/unitree_sdk2:${LD_LIBRARY_PATH}"
export LANG="${LANG:-en_US.UTF-8}"
export LC_CTYPE="${LC_CTYPE:-${LANG}}"
if [ -z "${QT_IM_MODULE:-}" ]; then
    case "${DISPLAY:-}" in
        localhost:*|127.0.0.1:*)
            export QT_IM_MODULE="xim"
            ;;
        *)
            export QT_IM_MODULE="ibus"
            ;;
    esac
fi
export GTK_IM_MODULE="${GTK_IM_MODULE:-$QT_IM_MODULE}"
case "$QT_IM_MODULE" in
    fcitx)
        export XMODIFIERS="${XMODIFIERS:-@im=fcitx}"
        ;;
    ibus)
        export XMODIFIERS="${XMODIFIERS:-@im=ibus}"
        ;;
    *)
        export XMODIFIERS="${XMODIFIERS:-@im=ibus}"
        ;;
esac

if command -v ibus-daemon >/dev/null 2>&1 && ! pgrep -u "$USER" -x ibus-daemon >/dev/null 2>&1; then
    ibus-daemon -drx >/tmp/hongtu_ibus.log 2>&1 || true
fi

if [ -f /opt/ros/noetic/setup.bash ]; then
    source /opt/ros/noetic/setup.bash
fi
if [ -f "${BASE}/G1Nav2D/devel/setup.bash" ]; then
    source "${BASE}/G1Nav2D/devel/setup.bash"
fi

echo "[robot] HongTu: $BASE"
echo "[robot] config: $HONGTU_CONFIG_FILE"
echo "[robot] DDS net_if: $HONGTU_G1_NET_IF"
echo "[robot] CYCLONEDDS_HOME: $CYCLONEDDS_HOME"
echo "[robot] QT_IM_MODULE: $QT_IM_MODULE"

cd "$BASE"

PID_R=""
PID_L=""
if python3 - <<'PY'
import pymodbus
PY
then
    echo "[1/3] 启动右手驱动 (192.168.123.211)…"
    python3 inspire_hand_driver.py --lr r --tcp-ip 192.168.123.211 --network "$HONGTU_G1_NET_IF" &
    PID_R=$!

    echo "[2/3] 启动左手驱动 (192.168.123.210)…"
    python3 inspire_hand_driver.py --lr l --tcp-ip 192.168.123.210 --network "$HONGTU_G1_NET_IF" &
    PID_L=$!
else
    echo "[1/3] 跳过灵巧手驱动：未安装 pymodbus"
    echo "[2/3] 跳过灵巧手驱动：未安装 pymodbus"
fi

cleanup() {
    [ -n "$PID_R" ] && kill "$PID_R" 2>/dev/null || true
    [ -n "$PID_L" ] && kill "$PID_L" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

sleep 2

echo "[3/3] 启动 G1 本体 GUI…"
cd g1_nav_panel
python3 main.py
