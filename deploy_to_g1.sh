#!/bin/bash
# Deploy HongTu to an isolated directory on the G1 body over the wired link.

set -e

G1_HOST="${G1_HOST:-192.168.123.164}"
G1_USER="${G1_USER:-unitree}"
G1_DIR="${G1_DIR:-/home/unitree/zgx_g1}"
REMOTE="${G1_USER}@${G1_HOST}"

BASE="$(cd "$(dirname "$0")" && pwd)"

echo "[deploy] target: ${REMOTE}:${G1_DIR}"
echo "[deploy] source: ${BASE}"

ssh "$REMOTE" "mkdir -p '${G1_DIR}' '${G1_DIR}/.runtime'"

rsync -az --delete \
    --exclude='.git/' \
    --exclude='__pycache__/' \
    --exclude='*.pyc' \
    --exclude='.runtime/' \
    --exclude='G1Nav2D/build/' \
    --exclude='G1Nav2D/devel/' \
    "$BASE/" "${REMOTE}:${G1_DIR}/"

ssh "$REMOTE" "cd '${G1_DIR}' && chmod +x start_robot_gui.sh start_g1_backend.sh start_pc_remote_gui.sh deploy_to_g1.sh remote_robot_gui.sh g1_robot_service.py g1_remote_client.py"

echo "[deploy] done"
echo "[deploy] remote start:"
echo "  ssh ${REMOTE} 'cd ${G1_DIR} && ./start_g1_backend.sh'"
