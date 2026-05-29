#!/bin/bash
# 一键启动双手灵巧手驱动 + G1 控制面板
cd ~/Desktop/HongTu

echo "[1/3] 启动右手驱动 (192.168.123.211)…"
python3 inspire_hand_driver.py --lr r --tcp-ip 192.168.123.211 &
PID_R=$!

echo "[2/3] 启动左手驱动 (192.168.123.210)…"
python3 inspire_hand_driver.py --lr l --tcp-ip 192.168.123.210 &
PID_L=$!

sleep 2

echo "[3/3] 启动 G1 控制面板…"
cd g1_nav_panel
python3 main.py

# GUI 关闭后清理驱动进程
kill $PID_R $PID_L 2>/dev/null
echo "已关闭所有驱动"
