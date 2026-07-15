"""Stateful keyboard and mouse report dispatch for the host bridge."""

from __future__ import annotations

from typing import Protocol

from .protocol import encode_keyboard_report, encode_mouse_report


class PacketSink(Protocol):
    def send(self, packet: bytes) -> None: ...


class RemoteInputController:
    def __init__(self, sink: PacketSink) -> None:
        self._sink = sink
        self._active = False
        self._keys: set[int] = set()
        self._modifiers = 0
        self._mouse_buttons = 0

    @property
    def active(self) -> bool:
        return self._active

    def toggle_remote_mode(self) -> bool:
        self._active = not self._active
        if not self._active:
            self._keys.clear()
            self._modifiers = 0
            self._mouse_buttons = 0
            self._send_keyboard()
            self._send_mouse(0, 0, 0)
        return self._active

    def key_down(self, usage: int, modifier: int = 0) -> None:
        if not self._active:
            return
        if modifier:
            self._modifiers |= modifier
        else:
            self._keys.add(usage)
        self._send_keyboard()

    def key_up(self, usage: int, modifier: int = 0) -> None:
        if not self._active:
            return
        if modifier:
            self._modifiers &= ~modifier
        else:
            self._keys.discard(usage)
        self._send_keyboard()

    def mouse_move(self, dx: int, dy: int) -> None:
        if self._active and (dx or dy):
            self._send_mouse(dx, dy, 0)

    def mouse_button(self, mask: int, pressed: bool) -> None:
        if not self._active:
            return
        if pressed:
            self._mouse_buttons |= mask
        else:
            self._mouse_buttons &= ~mask
        self._send_mouse(0, 0, 0)

    def mouse_scroll(self, wheel: int) -> None:
        if self._active and wheel:
            self._send_mouse(0, 0, wheel)

    def _send_keyboard(self) -> None:
        self._sink.send(encode_keyboard_report(self._modifiers, sorted(self._keys)))

    def _send_mouse(self, dx: int, dy: int, wheel: int) -> None:
        self._sink.send(encode_mouse_report(self._mouse_buttons, dx, dy, wheel))
