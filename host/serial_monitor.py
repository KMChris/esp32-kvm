#!/usr/bin/env python3
"""Read ESP32 KVM boot, BLE, and periodic status logs from the serial port."""

from __future__ import annotations

import argparse
import time

import serial

from kvm_bridge.serial_bridge import SerialPacketSink


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="ESP32 serial port; auto-detected when omitted")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--seconds", type=float, default=30.0)
    args = parser.parse_args()

    port = args.port or SerialPacketSink.detect_port()
    if not port:
        raise SystemExit("ESP32 serial port not found; pass --port explicitly")

    print(f"Monitoring {port} at {args.baud} baud for {args.seconds:g} seconds")
    deadline = time.monotonic() + args.seconds
    with serial.Serial(port, args.baud, timeout=0.2) as device:
        while time.monotonic() < deadline:
            text = device.read(device.in_waiting or 1).decode("utf-8", errors="replace")
            if text:
                print(text, end="", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
