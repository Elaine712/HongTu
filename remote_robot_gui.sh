#!/bin/bash
# Start the G1-body GUI from the PC through SSH X11 forwarding.

set -e

G1_WIRED_HOST="${G1_WIRED_HOST:-192.168.123.164}"
G1_WIFI_HOST="${G1_WIFI_HOST:-192.168.1.24}"
G1_HOST="${G1_HOST:-}"
G1_USER="${G1_USER:-unitree}"
G1_DIR="${G1_DIR:-/home/unitree/zgx_g1}"
SSH_CHECK_OPTS=(-o BatchMode=yes -o ConnectTimeout=2)

detect_im_module() {
    if [ -n "${QT_IM_MODULE:-}" ]; then
        echo "$QT_IM_MODULE"
    elif pgrep -x fcitx5 >/dev/null 2>&1; then
        echo "fcitx"
    elif pgrep -x fcitx >/dev/null 2>&1; then
        echo "fcitx"
    elif pgrep -x ibus-daemon >/dev/null 2>&1; then
        echo "ibus"
    else
        echo "ibus"
    fi
}

LOCAL_IM_MODULE="$(detect_im_module)"
export QT_IM_MODULE="${HONGTU_REMOTE_QT_IM_MODULE:-$LOCAL_IM_MODULE}"
export GTK_IM_MODULE="${GTK_IM_MODULE:-$QT_IM_MODULE}"
if [ "$LOCAL_IM_MODULE" = "fcitx" ]; then
    export XMODIFIERS="${XMODIFIERS:-@im=fcitx}"
else
    export XMODIFIERS="${XMODIFIERS:-@im=ibus}"
fi
export LC_CTYPE="${LC_CTYPE:-${LANG:-en_US.UTF-8}}"

can_ssh() {
    local host="$1"
    ssh "${SSH_CHECK_OPTS[@]}" "${G1_USER}@${host}" "true" >/dev/null 2>&1
}

wifi_prefixes() {
    ip -o -4 addr show up 2>/dev/null | awk '
        $2 ~ /^(wl|wlan)/ {
            split($4, a, "/");
            split(a[1], b, ".");
            if (b[1] && b[2] && b[3]) print b[1]"."b[2]"."b[3]
        }' | sort -u
}

discover_g1_host() {
    if [ -n "$G1_HOST" ]; then
        echo "$G1_HOST"
        return 0
    fi

    echo "[remote] checking wired ${G1_WIRED_HOST}..." >&2
    if can_ssh "$G1_WIRED_HOST"; then
        echo "$G1_WIRED_HOST"
        return 0
    fi

    echo "[remote] wired unavailable, checking WiFi ${G1_WIFI_HOST}..." >&2
    if can_ssh "$G1_WIFI_HOST"; then
        echo "$G1_WIFI_HOST"
        return 0
    fi

    for prefix in $(wifi_prefixes); do
        for suffix in 24 164; do
            host="${prefix}.${suffix}"
            [ "$host" = "$G1_WIFI_HOST" ] && continue
            [ "$host" = "$G1_WIRED_HOST" ] && continue
            echo "[remote] probing ${host}..." >&2
            if can_ssh "$host"; then
                echo "$host"
                return 0
            fi
        done
    done

    echo "[remote] 未找到可 SSH 的 G1。可手动指定：G1_HOST=192.168.1.24 ./remote_robot_gui.sh" >&2
    return 1
}

G1_HOST="$(discover_g1_host)"
REMOTE="${G1_USER}@${G1_HOST}"
echo "[remote] starting GUI on ${REMOTE}:${G1_DIR}"
echo "[remote] input method: QT_IM_MODULE=${QT_IM_MODULE}, XMODIFIERS=${XMODIFIERS}"
echo "[remote] if Chinese input still fails, try: HONGTU_REMOTE_QT_IM_MODULE=xim ./remote_robot_gui.sh"
printf -v REMOTE_CMD \
    "export LANG=%q LC_CTYPE=%q QT_IM_MODULE=%q GTK_IM_MODULE=%q XMODIFIERS=%q; cd %q && ./start_robot_gui.sh" \
    "${LANG:-en_US.UTF-8}" "$LC_CTYPE" "$QT_IM_MODULE" "$GTK_IM_MODULE" "$XMODIFIERS" "$G1_DIR"
ssh -Y -C \
    -o ForwardX11Trusted=yes \
    -o SendEnv=LANG \
    -o SendEnv=LC_CTYPE \
    -o SendEnv=LC_ALL \
    -o SendEnv=QT_IM_MODULE \
    -o SendEnv=GTK_IM_MODULE \
    -o SendEnv=XMODIFIERS \
    "$REMOTE" "$REMOTE_CMD"
