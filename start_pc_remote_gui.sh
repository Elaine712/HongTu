#!/bin/bash
# Run on the PC. Starts the GUI locally and sends G1 commands to the body backend.

set -e

BASE="$(cd "$(dirname "$0")" && pwd)"
G1_WIRED_HOST="${G1_WIRED_HOST:-192.168.123.164}"
G1_WIFI_HOST="${G1_WIFI_HOST:-192.168.1.24}"
G1_BACKEND_HOST="${G1_BACKEND_HOST:-${G1_HOST:-}}"
G1_USER="${G1_USER:-unitree}"
G1_BACKEND_PORT="${G1_BACKEND_PORT:-5055}"
G1_TUNNEL_PORT="${G1_TUNNEL_PORT:-15055}"
SSH_CHECK_OPTS=(-o BatchMode=yes -o ConnectTimeout=2)
TUNNEL_PID=""
BACKEND_ADDR=""
export NO_PROXY="${NO_PROXY:-localhost,127.0.0.1,192.168.0.0/16,10.0.0.0/8}"
export no_proxy="${no_proxy:-$NO_PROXY}"

can_http() {
    local host="$1"
    local port="${2:-$G1_BACKEND_PORT}"
    python3 - "$host" "$port" <<'PY'
import json
import sys
import urllib.request
host, port = sys.argv[1], sys.argv[2]
try:
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with opener.open(f"http://{host}:{port}/api/status", timeout=1.5) as resp:
        data = json.loads(resp.read().decode("utf-8"))
    raise SystemExit(0 if data.get("ok") else 1)
except Exception:
    raise SystemExit(1)
PY
}

can_ssh() {
    local host="$1"
    ssh "${SSH_CHECK_OPTS[@]}" "${G1_USER}@${host}" "true" >/dev/null 2>&1
}

start_tunnel() {
    local host="$1"
    echo "[pc-gui] direct HTTP blocked, opening SSH tunnel ${G1_TUNNEL_PORT}->${host}:${G1_BACKEND_PORT}..." >&2
    ssh "${SSH_CHECK_OPTS[@]}" -o ExitOnForwardFailure=yes -N \
        -L "127.0.0.1:${G1_TUNNEL_PORT}:127.0.0.1:${G1_BACKEND_PORT}" \
        "${G1_USER}@${host}" &
    TUNNEL_PID=$!
    for _ in $(seq 1 20); do
        if can_http "127.0.0.1" "$G1_TUNNEL_PORT"; then
            BACKEND_ADDR="127.0.0.1:${G1_TUNNEL_PORT}"
            return 0
        fi
        sleep 0.2
    done
    echo "[pc-gui] SSH tunnel started but backend did not respond" >&2
    return 1
}

wifi_prefixes() {
    ip -o -4 addr show up 2>/dev/null | awk '
        $2 ~ /^(wl|wlan)/ {
            split($4, a, "/");
            split(a[1], b, ".");
            if (b[1] && b[2] && b[3]) print b[1]"."b[2]"."b[3]
        }' | sort -u
}

discover_backend() {
    if [ -n "$G1_BACKEND_HOST" ]; then
        if can_http "$G1_BACKEND_HOST"; then
            BACKEND_ADDR="${G1_BACKEND_HOST}:${G1_BACKEND_PORT}"
            return 0
        fi
        if can_ssh "$G1_BACKEND_HOST"; then
            start_tunnel "$G1_BACKEND_HOST"
            return 0
        fi
        echo "[pc-gui] 指定的 G1_BACKEND_HOST 不可访问: $G1_BACKEND_HOST" >&2
        return 1
    fi
    for host in "$G1_WIRED_HOST" "$G1_WIFI_HOST"; do
        echo "[pc-gui] checking backend ${host}:${G1_BACKEND_PORT}..." >&2
        if can_http "$host"; then
            BACKEND_ADDR="${host}:${G1_BACKEND_PORT}"
            return 0
        fi
        if can_ssh "$host"; then
            start_tunnel "$host"
            return 0
        fi
    done
    for prefix in $(wifi_prefixes); do
        for suffix in 24 164; do
            host="${prefix}.${suffix}"
            echo "[pc-gui] probing backend ${host}:${G1_BACKEND_PORT}..." >&2
            if can_http "$host"; then
                BACKEND_ADDR="${host}:${G1_BACKEND_PORT}"
                return 0
            fi
            if can_ssh "$host"; then
                start_tunnel "$host"
                return 0
            fi
        done
    done
    echo "[pc-gui] 未找到 G1 后台服务。先在 G1 上运行: cd /home/unitree/zgx_g1 && ./start_g1_backend.sh" >&2
    return 1
}

cleanup() {
    if [ -n "$TUNNEL_PID" ]; then
        kill "$TUNNEL_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

discover_backend
export HONGTU_G1_BACKEND_URL="http://${BACKEND_ADDR}"
export HONGTU_REMOTE_AUTO_CONNECT="${HONGTU_REMOTE_AUTO_CONNECT:-1}"

echo "[pc-gui] backend: $HONGTU_G1_BACKEND_URL"
if [ "${HONGTU_PC_GUI_DRY_RUN:-0}" = "1" ]; then
    echo "[pc-gui] dry run ok"
    exit 0
fi
cd "$BASE/g1_nav_panel"
python3 main.py
