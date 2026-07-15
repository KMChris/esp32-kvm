"""USB HID usage translation independent of the host operating system."""

from __future__ import annotations

_LETTERS = {chr(ord("a") + index): 0x04 + index for index in range(26)}
_DIGITS = {str(value): 0x1E + value - 1 for value in range(1, 10)} | {"0": 0x27}

_CHAR_USAGES = {
    **_LETTERS,
    **_DIGITS,
    " ": 0x2C,
    "-": 0x2D,
    "=": 0x2E,
    "[": 0x2F,
    "]": 0x30,
    "\\": 0x31,
    ";": 0x33,
    "'": 0x34,
    "`": 0x35,
    ",": 0x36,
    ".": 0x37,
    "/": 0x38,
    "!": 0x1E,
    "@": 0x1F,
    "#": 0x20,
    "$": 0x21,
    "%": 0x22,
    "^": 0x23,
    "&": 0x24,
    "*": 0x25,
    "(": 0x26,
    ")": 0x27,
}

_NAMED_USAGES = {
    "backspace": 0x2A,
    "tab": 0x2B,
    "enter": 0x28,
    "esc": 0x29,
    "space": 0x2C,
    "caps_lock": 0x39,
    "scroll_lock": 0x47,
    "pause": 0x48,
    "insert": 0x49,
    "home": 0x4A,
    "page_up": 0x4B,
    "delete": 0x4C,
    "end": 0x4D,
    "page_down": 0x4E,
    "right": 0x4F,
    "left": 0x50,
    "down": 0x51,
    "up": 0x52,
}
_NAMED_USAGES.update({f"f{number}": 0x3A + number - 1 for number in range(1, 13)})
_NAMED_USAGES.update({f"f{number}": 0x68 + number - 13 for number in range(13, 25)})

_MODIFIERS = {
    "ctrl": 0x01,
    "ctrl_l": 0x01,
    "ctrl_r": 0x10,
    "shift": 0x02,
    "shift_l": 0x02,
    "shift_r": 0x20,
    "alt": 0x04,
    "alt_l": 0x04,
    "alt_r": 0x40,
    "cmd": 0x08,
    "cmd_l": 0x08,
    "cmd_r": 0x80,
}


def usage_for_char(value: str) -> int:
    return _CHAR_USAGES.get(value.lower() if value.isalpha() else value, 0)


def usage_for_name(name: str) -> int:
    return _NAMED_USAGES.get(name.lower(), 0)


def modifier_for_name(name: str) -> int:
    return _MODIFIERS.get(name.lower(), 0)
