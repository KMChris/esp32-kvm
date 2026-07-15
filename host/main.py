#!/usr/bin/env python3
"""Cross-platform Python source bridge for the ESP32 BLE KVM firmware."""

from __future__ import annotations

import argparse
from typing import Any

from kvm_bridge.controller import RemoteInputController
from kvm_bridge.keys import modifier_for_name, usage_for_char, usage_for_name
from kvm_bridge.serial_bridge import SerialPacketSink
from kvm_bridge.toggle import ToggleChord

MOUSE_BUTTONS = {"left": 0x01, "right": 0x02, "middle": 0x04, "x1": 0x08, "x2": 0x10}
DEFAULT_TOGGLE = "ctrl+alt+cmd+f"


def key_translation(key: Any) -> tuple[int, int]:
    """Return (HID usage, modifier bit) for a pynput key object."""
    char = getattr(key, "char", None)
    if char:
        return usage_for_char(char), 0
    name = getattr(key, "name", "")
    return usage_for_name(name), modifier_for_name(name)


def key_name(key: Any) -> str:
    char = getattr(key, "char", None)
    if char:
        return char.lower()
    return getattr(key, "name", "").lower()


class PynputCapture:
    def __init__(self, controller: RemoteInputController, toggle_spec: str) -> None:
        self._controller = controller
        self._toggle_spec = toggle_spec
        self._toggle = ToggleChord.parse(toggle_spec)
        self._last_pointer: tuple[int, int] | None = None

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
            self._run_remote(keyboard, mouse)

    def _activate(self, key: Any) -> bool | None:
        if self._toggle.press(key_name(key)):
            self._controller.toggle_remote_mode()
            print(f"[REMOTE] Active. Hold {self._toggle_spec} again to return local control.")
            return False
        return None

    def _toggle_release(self, key: Any) -> None:
        self._toggle.release(key_name(key))

    def _run_remote(self, keyboard: Any, mouse: Any) -> None:
        self._toggle.clear()
        self._last_pointer = None
        with keyboard.Listener(on_press=self._key_press,
                               on_release=self._key_release, suppress=True) as keys:
            with mouse.Listener(on_move=self._mouse_move, on_click=self._mouse_click,
                                on_scroll=self._mouse_scroll, suppress=True) as pointer:
                keys.join()
                pointer.stop()
        if self._controller.active:
            self._controller.toggle_remote_mode()
        print("[LOCAL] Control returned to source computer.")

    def _key_press(self, key: Any) -> bool | None:
        if self._toggle.press(key_name(key)):
            return False
        usage, modifier = key_translation(key)
        if usage or modifier:
            self._controller.key_down(usage, modifier)
        return None

    def _key_release(self, key: Any) -> None:
        self._toggle.release(key_name(key))
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


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="ESP32 serial port; auto-detected when unambiguous")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--toggle", default=DEFAULT_TOGGLE,
                        help="simultaneous '+'-separated toggle chord; defaults to ctrl+alt+cmd+f")
    args = parser.parse_args()

    port = args.port or SerialPacketSink.detect_port()
    if not port:
        raise SystemExit("ESP32 serial port not found; pass --port /dev/cu.usbserial-... explicitly")

    sink = SerialPacketSink(port, args.baud)
    sink.open()
    print(f"[READY] Serial bridge connected: {port} at {args.baud} baud")
    try:
        PynputCapture(RemoteInputController(sink), args.toggle).run()
    except KeyboardInterrupt:
        print("\n[STOP] Bridge stopped")
    finally:
        sink.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
