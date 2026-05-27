#!/usr/bin/env python3
"""Inspire 灵巧手 Modbus 驱动进程 (后台运行)"""
import sys, os, time

BASE = os.path.dirname(os.path.abspath(__file__))
for p in [os.path.join(BASE, "unitree_sdk2_python"), os.path.join(BASE, "inspire_hand")]:
    if os.path.isdir(p) and p not in sys.path:
        sys.path.insert(0, p)

from inspire_sdkpy import inspire_sdk, inspire_hand_defaut

import argparse

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--lr", default="l", choices=["l", "r"])
    parser.add_argument("--serial", default="/dev/ttyUSB0")
    parser.add_argument("--device-id", type=int, default=None,
                        help="Modbus device ID (默认: l=2, r=1)")
    parser.add_argument("--tcp-ip", default=None, help="使用 TCP 而非串口")
    parser.add_argument("--network", default=None,
                        help="DDS 网卡名称 (与 G1 导航面板的网卡一致，如 eno1)")
    args = parser.parse_args()

    if args.device_id is None:
        args.device_id = 2 if args.lr == "l" else 1

    states_structure = [
        ('angle_act', 1546, 6, 'short'),
        ('force_act', 1582, 6, 'short'),
        ('status', 1612, 3, 'byte'),
    ]

    use_serial = args.tcp_ip is None
    handler = inspire_sdk.ModbusDataHandler(
        LR=args.lr,
        device_id=args.device_id,
        use_serial=use_serial,
        serial_port=args.serial,
        ip=args.tcp_ip,
        network=args.network,
        states_structure=states_structure,
        initDDS=True,
    )

    print(f"驱动启动: hand={args.lr}, device_id={args.device_id}, "
          f"{'TCP '+args.tcp_ip if not use_serial else 'Serial '+args.serial}")

    count = 0
    t0 = time.perf_counter()
    try:
        while True:
            try:
                handler.read()
            except Exception as e:
                pass
            count += 1
            if count % 200 == 0:
                hz = count / (time.perf_counter() - t0)
                print(f"{args.lr} hand: {hz:.1f} Hz")
            time.sleep(0.005)
    except KeyboardInterrupt:
        print("驱动停止")

if __name__ == "__main__":
    main()
