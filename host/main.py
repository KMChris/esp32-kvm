#!/usr/bin/env python3
"""Cross-platform Python source bridge for the ESP32 BLE KVM firmware."""

from __future__ import annotations

import argparse
import os
import sys
from typing import Any

from kvm_bridge.controller import RemoteInputController
from kvm_bridge.keys import modifier_for_name, usage_for_char, usage_for_name, usage_for_vk
from kvm_bridge.serial_bridge import SerialPacketSink
from kvm_bridge.toggle import ToggleChord
from kvm_bridge.udp_bridge import UdpPacketSink

MOUSE_BUTTONS = {"left": 0x01, "right": 0x02, "middle": 0x04, "x1": 0x08, "x2": 0x10}
DEFAULT_TOGGLE = "ctrl+alt+cmd+f"


def key_translation(key: Any) -> tuple[int, int]:
    """Return (HID usage, modifier bit) for a pynput key object."""
    char = getattr(key, "char", None)
    if char:
        usage = usage_for_char(char)
        if usage:
            return usage, 0
    name = getattr(key, "name", "")
    usage = usage_for_name(name)
    modifier = modifier_for_name(name)
    if usage or modifier:
        return usage, modifier
    return usage_for_vk(getattr(key, "vk", None)), 0


def key_name(key: Any) -> str:
    char = getattr(key, "char", None)
    if char:
        return char.lower()
    return getattr(key, "name", "").lower()


def restart_argv(executable: str, args: list[str]) -> list[str]:
    return [executable, *args]


def udp_port(value: str) -> int:
    port = int(value)
    if not 1 <= port <= 65535:
        raise argparse.ArgumentTypeError("must be between 1 and 65535")
    return port


class PynputCapture:
    def __init__(self, controller: RemoteInputController, toggle_spec: str) -> None:
        self._controller = controller
        self._toggle_spec = toggle_spec
        self._toggle = ToggleChord.parse(toggle_spec)
        self._last_pointer: tuple[int, int] | None = None
        self._exit_requested = False
        self._exit_key = ""
        self._stop_after_return = False

    def run(self) -> None:
        try:
            from pynput import keyboard, mouse
        except ImportError as exc:
            raise SystemExit("Install host requirements: python3 -m pip install -r host/requirements.txt") from exc

        print(f"[LOCAL] Hold {self._toggle_spec} to enable remote mode. Ctrl-C exits.")
        while True:
            self._toggle.clear()
            with keyboard.Listener(on_press=self._activate, on_release=self._toggle_release) as listener:
                listener.join()
            if self._run_remote(keyboard, mouse):
                return

    def _activate(self, key: Any) -> bool | None:
        if self._toggle.press(key_name(key)):
            self._controller.toggle_remote_mode()
            print(f"[REMOTE] Active. Hold {self._toggle_spec} again to return local control.")
            return False
        return None

    def _toggle_release(self, key: Any) -> None:
        self._toggle.release(key_name(key))

    def _run_remote(self, keyboard: Any, mouse: Any) -> bool:
        self._toggle.clear()
        self._last_pointer = None
        self._exit_requested = False
        self._exit_key = ""
        self._stop_after_return = False
        with keyboard.Listener(on_press=self._key_press,
                               on_release=self._key_release, suppress=True) as keys:
            with mouse.Listener(on_move=self._mouse_move, on_click=self._mouse_click,
                                on_scroll=self._mouse_scroll, suppress=True) as pointer:
                keys.join()
                pointer.stop()
                pointer.join()
        if self._controller.active:
            self._controller.toggle_remote_mode()
        print("[LOCAL] Control returned to source computer.")
        return self._stop_after_return

    def _key_press(self, key: Any) -> bool | None:
        if self._exit_requested:
            return None
        name = key_name(key)
        if self._toggle.press(name):
            self._exit_requested = True
            self._exit_key = name
            self._stop_after_return = True
            self._controller.toggle_remote_mode()
            return None
        usage, modifier = key_translation(key)
        if usage or modifier:
            self._controller.key_down(usage, modifier)
        return None

    def _key_release(self, key: Any) -> bool | None:
        name = key_name(key)
        self._toggle.release(name)
        if self._exit_requested:
            return False if name == self._exit_key else None
        usage, modifier = key_translation(key)
        if usage or modifier:
            self._controller.key_up(usage, modifier)

    def _mouse_move(self, x: int, y: int) -> None:
        if self._last_pointer is not None:
            old_x, old_y = self._last_pointer
            self._controller.mouse_move(x - old_x, y - old_y)
        self._last_pointer = (x, y)

    def _mouse_click(self, _x: int, _y: int, button: Any, pressed: bool) -> None:
        mask = MOUSE_BUTTONS.get(getattr(button, "name", ""), 0)
        if mask:
            self._controller.mouse_button(mask, pressed)

    def _mouse_scroll(self, _x: int, _y: int, _dx: int, dy: int) -> None:
        self._controller.mouse_scroll(dy)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--transport", choices=("serial", "udp"), default="serial",
                        help="source-to-ESP32 transport; defaults to serial")
    parser.add_argument("--port", help="ESP32 serial port; auto-detected when unambiguous")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--udp-host", default="192.168.4.1",
                        help="ESP32 SoftAP gateway IP for --transport udp")
    parser.add_argument("--udp-port", type=udp_port, default=3333,
                        help="ESP32 UDP input port for --transport udp")
    parser.add_argument("--toggle", default=DEFAULT_TOGGLE,
                        help="simultaneous '+'-separated toggle chord; defaults to ctrl+alt+cmd+f")
    return parser.parse_args(argv)


def main() -> int:
    args = parse_args()
    if args.transport == "serial":
        port = args.port or SerialPacketSink.detect_port()
        if not port:
            raise SystemExit("ESP32 serial port not found; pass --port /dev/cu.usbserial-... explicitly")
        sink = SerialPacketSink(port, args.baud)
        description = f"Serial bridge connected: {port} at {args.baud} baud"
    else:
        sink = UdpPacketSink(args.udp_host, args.udp_port)
        description = f"UDP bridge connected: {args.udp_host}:{args.udp_port}"
    sink.open()
    print(f"[READY] {description}")
    restart = False
    try:
        PynputCapture(RemoteInputController(sink), args.toggle).run()
        restart = True
    except KeyboardInterrupt:
        print("\n[STOP] Bridge stopped")
    finally:
        sink.close()
    if restart:
        print("[LOCAL] Restarting bridge for the next remote session.")
        os.execv(sys.executable, restart_argv(sys.executable, sys.argv))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
