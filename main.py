import ctypes
import threading
import time
from abc import ABC, abstractmethod
from contextlib import contextmanager
from ctypes import wintypes
from dataclasses import dataclass, field
from enum import IntEnum, IntFlag

import serial
import serial.tools.list_ports
from pynput import keyboard, mouse


class WindowsMessage(IntEnum):
    INPUT = 0x00FF
    DESTROY = 0x0002
    QUIT = 0x0012


class RawInputFlag(IntFlag):
    INPUT_SINK = 0x00000100


class RawInputType(IntEnum):
    MOUSE = 0
    KEYBOARD = 1


class RawInputCommand(IntEnum):
    INPUT = 0x10000003


class KeyboardFlag(IntFlag):
    BREAK = 1
    E0 = 2
    E1 = 4


class HidUsage(IntEnum):
    GENERIC_DESKTOP = 0x01
    MOUSE = 0x02
    KEYBOARD = 0x06


class MouseButton(IntFlag):
    LEFT = 1
    RIGHT = 2
    MIDDLE = 4


class KeyboardModifier(IntFlag):
    CTRL_LEFT = 0x01
    SHIFT_LEFT = 0x02
    ALT_LEFT = 0x04
    GUI_LEFT = 0x08
    CTRL_RIGHT = 0x10
    SHIFT_RIGHT = 0x20
    ALT_RIGHT = 0x40
    GUI_RIGHT = 0x80


SCANCODE_TO_HID = {
    0x1E: 0x04, 0x30: 0x05, 0x2E: 0x06, 0x20: 0x07, 0x12: 0x08, 0x21: 0x09,
    0x22: 0x0A, 0x23: 0x0B, 0x17: 0x0C, 0x24: 0x0D, 0x25: 0x0E, 0x26: 0x0F,
    0x32: 0x10, 0x31: 0x11, 0x18: 0x12, 0x19: 0x13, 0x10: 0x14, 0x13: 0x15,
    0x1F: 0x16, 0x14: 0x17, 0x16: 0x18, 0x2F: 0x19, 0x11: 0x1A, 0x2D: 0x1B,
    0x15: 0x1C, 0x2C: 0x1D,
    0x02: 0x1E, 0x03: 0x1F, 0x04: 0x20, 0x05: 0x21, 0x06: 0x22,
    0x07: 0x23, 0x08: 0x24, 0x09: 0x25, 0x0A: 0x26, 0x0B: 0x27,
    0x1C: 0x28, 0x01: 0x29, 0x0E: 0x2A, 0x0F: 0x2B, 0x39: 0x2C,
    0x0C: 0x2D, 0x0D: 0x2E, 0x1A: 0x2F, 0x1B: 0x30, 0x2B: 0x31,
    0x27: 0x33, 0x28: 0x34, 0x29: 0x35, 0x33: 0x36, 0x34: 0x37, 0x35: 0x38,
    0x3B: 0x3A, 0x3C: 0x3B, 0x3D: 0x3C, 0x3E: 0x3D, 0x3F: 0x3E, 0x40: 0x3F,
    0x41: 0x40, 0x42: 0x41, 0x43: 0x42, 0x44: 0x43, 0x57: 0x44, 0x58: 0x45,
    0x3A: 0x39, 0x46: 0x47, 0x45: 0x53,
    0x52: 0x62, 0x4F: 0x59, 0x50: 0x5A, 0x51: 0x5B, 0x4B: 0x5C,
    0x4C: 0x5D, 0x4D: 0x5E, 0x47: 0x5F, 0x48: 0x60, 0x49: 0x61,
    0x37: 0x55, 0x4E: 0x57, 0x4A: 0x56, 0x53: 0x63,
}

SCANCODE_TO_HID_EXTENDED = {
    0x1C: 0x58, 0x35: 0x54, 0x52: 0x49, 0x47: 0x4A, 0x49: 0x4B,
    0x53: 0x4C, 0x4F: 0x4D, 0x51: 0x4E, 0x4D: 0x4F, 0x4B: 0x50,
    0x50: 0x51, 0x48: 0x52, 0x5D: 0x65, 0x38: 0x00, 0x1D: 0x00,
    0x5B: 0x00, 0x5C: 0x00, 0x37: 0x46,
}

SCANCODE_TO_MODIFIER = {
    0x1D: KeyboardModifier.CTRL_LEFT,
    0x2A: KeyboardModifier.SHIFT_LEFT,
    0x36: KeyboardModifier.SHIFT_RIGHT,
    0x38: KeyboardModifier.ALT_LEFT,
}

SCANCODE_TO_MODIFIER_EXTENDED = {
    0x1D: KeyboardModifier.CTRL_RIGHT,
    0x38: KeyboardModifier.ALT_RIGHT,
    0x5B: KeyboardModifier.GUI_LEFT,
    0x5C: KeyboardModifier.GUI_RIGHT,
}

PYNPUT_TO_MOUSE_BUTTON = {
    mouse.Button.left: MouseButton.LEFT,
    mouse.Button.right: MouseButton.RIGHT,
    mouse.Button.middle: MouseButton.MIDDLE,
}


@dataclass
class BridgeConfig:
    port: str = ""
    baudrate: int = 115200
    sensitivity: float = 1.0
    poll_rate_hz: int = 100
    max_delta: int = 120
    reconnect_delay_seconds: float = 2.0
    reconnect_max_attempts: int = 5
    toggle_scan_code: int = 0x1D
    toggle_use_e1_prefix: bool = True


@dataclass
class InputState:
    pressed_keys: set = field(default_factory=set)
    active_modifiers: int = 0
    mouse_buttons: int = 0
    accumulated_dx: float = 0.0
    accumulated_dy: float = 0.0
    accumulated_wheel: float = 0.0
    last_transmission_time: float = 0.0

    def reset(self):
        self.pressed_keys.clear()
        self.active_modifiers = 0
        self.mouse_buttons = 0
        self.accumulated_dx = 0.0
        self.accumulated_dy = 0.0
        self.accumulated_wheel = 0.0
        self.last_transmission_time = 0.0


_user32 = ctypes.windll.user32
_kernel32 = ctypes.windll.kernel32

_LONG_PTR = ctypes.c_longlong if ctypes.sizeof(ctypes.c_void_p) == 8 else ctypes.c_long
_WNDPROC = ctypes.WINFUNCTYPE(
    _LONG_PTR, wintypes.HWND, wintypes.UINT, wintypes.WPARAM, wintypes.LPARAM
)

_user32.DefWindowProcW.argtypes = [
    wintypes.HWND, wintypes.UINT, wintypes.WPARAM, wintypes.LPARAM
]
_user32.DefWindowProcW.restype = _LONG_PTR


class _RAWINPUTDEVICE(ctypes.Structure):
    _fields_ = [
        ("usUsagePage", wintypes.USHORT),
        ("usUsage", wintypes.USHORT),
        ("dwFlags", wintypes.DWORD),
        ("hwndTarget", wintypes.HWND),
    ]


class _RAWINPUTHEADER(ctypes.Structure):
    _fields_ = [
        ("dwType", wintypes.DWORD),
        ("dwSize", wintypes.DWORD),
        ("hDevice", wintypes.HANDLE),
        ("wParam", wintypes.WPARAM),
    ]


class _RAWMOUSE(ctypes.Structure):
    _fields_ = [
        ("usFlags", wintypes.USHORT),
        ("usButtonFlags", wintypes.USHORT),
        ("usButtonData", wintypes.USHORT),
        ("ulRawButtons", wintypes.ULONG),
        ("lLastX", wintypes.LONG),
        ("lLastY", wintypes.LONG),
        ("ulExtraInformation", wintypes.ULONG),
    ]


class _RAWKEYBOARD(ctypes.Structure):
    _fields_ = [
        ("MakeCode", wintypes.USHORT),
        ("Flags", wintypes.USHORT),
        ("Reserved", wintypes.USHORT),
        ("VKey", wintypes.USHORT),
        ("Message", wintypes.UINT),
        ("ExtraInformation", wintypes.ULONG),
    ]


class _RAWINPUT_MOUSE(ctypes.Structure):
    _fields_ = [
        ("header", _RAWINPUTHEADER),
        ("mouse", _RAWMOUSE),
    ]


class _RAWINPUT_KEYBOARD(ctypes.Structure):
    _fields_ = [
        ("header", _RAWINPUTHEADER),
        ("keyboard", _RAWKEYBOARD),
    ]


class _WNDCLASS(ctypes.Structure):
    _fields_ = [
        ("style", wintypes.UINT),
        ("lpfnWndProc", _WNDPROC),
        ("cbClsExtra", ctypes.c_int),
        ("cbWndExtra", ctypes.c_int),
        ("hInstance", wintypes.HINSTANCE),
        ("hIcon", wintypes.HICON),
        ("hCursor", wintypes.HCURSOR),
        ("hbrBackground", wintypes.HBRUSH),
        ("lpszMenuName", wintypes.LPCWSTR),
        ("lpszClassName", wintypes.LPCWSTR),
    ]


class SerialBridge:
    PROTOCOL_HEADER = 0xFD
    WAKE_SEQUENCE = b"WAKEUP"
    KEYBOARD_KEYS_COUNT = 6

    def __init__(self, config):
        self._config = config
        self._serial = None
        self._lock = threading.Lock()

    @property
    def is_connected(self):
        return self._serial is not None and self._serial.is_open

    def connect(self):
        with self._lock:
            self._close_connection_unsafe()

            for attempt in range(self._config.reconnect_max_attempts):
                try:
                    self._serial = serial.Serial(
                        port=self._config.port,
                        baudrate=self._config.baudrate,
                        timeout=0.1,
                        write_timeout=0.5,
                    )
                    self._send_wake_sequence()
                    return True
                except serial.SerialException:
                    if attempt < self._config.reconnect_max_attempts - 1:
                        time.sleep(self._config.reconnect_delay_seconds)

            return False

    def disconnect(self):
        with self._lock:
            self._close_connection_unsafe()

    def send_keyboard_report(self, modifiers, keys):
        padded_keys = (keys + [0] * self.KEYBOARD_KEYS_COUNT)[:self.KEYBOARD_KEYS_COUNT]
        packet = bytes([self.PROTOCOL_HEADER, modifiers, 0x00] + padded_keys)
        return self._transmit(packet)

    def send_mouse_report(self, buttons, delta_x, delta_y, delta_wheel):
        packet = bytes([
            self.PROTOCOL_HEADER,
            0x00,
            0x03,
            buttons,
            self._to_signed_byte(delta_x),
            self._to_signed_byte(delta_y),
            self._to_signed_byte(delta_wheel),
            0,
            0,
        ])
        return self._transmit(packet)

    def _close_connection_unsafe(self):
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None

    def _send_wake_sequence(self):
        if self._serial is None:
            return

        self._serial.dtr = False
        self._serial.rts = True
        time.sleep(0.1)
        self._serial.rts = False
        time.sleep(0.2)
        self._serial.write(self.WAKE_SEQUENCE)
        time.sleep(0.3)

    def _transmit(self, data):
        with self._lock:
            if not self.is_connected:
                return False
            try:
                self._serial.write(data)
                return True
            except serial.SerialException:
                self._close_connection_unsafe()
                return False

    @staticmethod
    def _to_signed_byte(value):
        clamped = max(-127, min(127, value))
        return clamped & 0xFF

    @staticmethod
    def find_esp32_port():
        known_chips = ("CP210", "CH34", "FTDI", "USB-SERIAL")

        for port_info in serial.tools.list_ports.comports():
            description = (port_info.description or "").upper()
            if any(chip in description for chip in known_chips):
                return port_info.device

        return None


class RawInputCapture(ABC):
    def __init__(self, window_class_name, usage):
        self._window_class_name = window_class_name
        self._usage = usage
        self._hwnd = None
        self._thread = None
        self._thread_id = None
        self._ready_event = threading.Event()
        self._wndproc_reference = None

    def start(self):
        self._thread = threading.Thread(target=self._message_loop, daemon=True)
        self._thread.start()
        self._ready_event.wait(timeout=2.0)

    def stop(self):
        if self._thread_id is not None:
            _user32.PostThreadMessageW(self._thread_id, WindowsMessage.QUIT, 0, 0)
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    @abstractmethod
    def _process_input(self, lparam):
        pass

    def _message_loop(self):
        self._thread_id = _kernel32.GetCurrentThreadId()
        h_instance = _kernel32.GetModuleHandleW(None)

        @ctypes.WINFUNCTYPE(
            _LONG_PTR, wintypes.HWND, wintypes.UINT, wintypes.WPARAM, wintypes.LPARAM
        )
        def window_procedure(hwnd, msg, wparam, lparam):
            if msg == WindowsMessage.INPUT:
                self._process_input(lparam)
                return 0
            if msg == WindowsMessage.DESTROY:
                _user32.PostQuitMessage(0)
                return 0
            return _user32.DefWindowProcW(hwnd, msg, wparam, lparam)

        self._wndproc_reference = window_procedure

        window_class = _WNDCLASS()
        window_class.lpfnWndProc = window_procedure
        window_class.hInstance = h_instance
        window_class.lpszClassName = self._window_class_name
        _user32.RegisterClassW(ctypes.byref(window_class))

        self._hwnd = _user32.CreateWindowExW(
            0, self._window_class_name, "", 0, 0, 0, 0, 0, None, None, h_instance, None
        )

        raw_input_device = _RAWINPUTDEVICE(
            HidUsage.GENERIC_DESKTOP,
            self._usage,
            RawInputFlag.INPUT_SINK,
            self._hwnd,
        )
        _user32.RegisterRawInputDevices(
            ctypes.byref(raw_input_device), 1, ctypes.sizeof(_RAWINPUTDEVICE)
        )

        self._ready_event.set()

        msg = wintypes.MSG()
        while _user32.GetMessageW(ctypes.byref(msg), None, 0, 0) > 0:
            _user32.TranslateMessage(ctypes.byref(msg))
            _user32.DispatchMessageW(ctypes.byref(msg))


class RawMouseCapture(RawInputCapture):
    def __init__(self, on_move):
        super().__init__("RawMouseWindow", HidUsage.MOUSE)
        self._on_move = on_move

    def _process_input(self, lparam):
        size = wintypes.UINT(0)
        _user32.GetRawInputData(
            lparam,
            RawInputCommand.INPUT,
            None,
            ctypes.byref(size),
            ctypes.sizeof(_RAWINPUTHEADER),
        )

        if size.value == 0:
            return

        buffer = (ctypes.c_byte * size.value)()
        _user32.GetRawInputData(
            lparam,
            RawInputCommand.INPUT,
            buffer,
            ctypes.byref(size),
            ctypes.sizeof(_RAWINPUTHEADER),
        )

        raw_input = ctypes.cast(buffer, ctypes.POINTER(_RAWINPUT_MOUSE)).contents

        if raw_input.header.dwType == RawInputType.MOUSE:
            delta_x = raw_input.mouse.lLastX
            delta_y = raw_input.mouse.lLastY
            if delta_x or delta_y:
                self._on_move(delta_x, delta_y)


class RawKeyboardCapture(RawInputCapture):
    def __init__(self, on_key):
        super().__init__("RawKeyboardWindow", HidUsage.KEYBOARD)
        self._on_key = on_key

    def _process_input(self, lparam):
        size = wintypes.UINT(0)
        _user32.GetRawInputData(
            lparam,
            RawInputCommand.INPUT,
            None,
            ctypes.byref(size),
            ctypes.sizeof(_RAWINPUTHEADER),
        )

        if size.value == 0:
            return

        buffer = (ctypes.c_byte * size.value)()
        _user32.GetRawInputData(
            lparam,
            RawInputCommand.INPUT,
            buffer,
            ctypes.byref(size),
            ctypes.sizeof(_RAWINPUTHEADER),
        )

        raw_input = ctypes.cast(buffer, ctypes.POINTER(_RAWINPUT_KEYBOARD)).contents

        if raw_input.header.dwType == RawInputType.KEYBOARD:
            scan_code = raw_input.keyboard.MakeCode
            flags = raw_input.keyboard.Flags
            is_pressed = (flags & KeyboardFlag.BREAK) == 0
            self._on_key(scan_code, flags, is_pressed)


class CursorManager:
    def __init__(self):
        self._saved_position = None

    def lock(self):
        point = wintypes.POINT()
        if _user32.GetCursorPos(ctypes.byref(point)):
            self._saved_position = (point.x, point.y)
            clip_rect = wintypes.RECT(point.x, point.y, point.x + 1, point.y + 1)
            _user32.ClipCursor(ctypes.byref(clip_rect))
        _user32.ShowCursor(False)

    def unlock(self):
        _user32.ClipCursor(None)
        if self._saved_position is not None:
            _user32.SetCursorPos(self._saved_position[0], self._saved_position[1])
            self._saved_position = None
        _user32.ShowCursor(True)

    @contextmanager
    def locked_context(self):
        self.lock()
        try:
            yield
        finally:
            self.unlock()


class KVMController:
    def __init__(self, config):
        self._config = config
        self._bridge = SerialBridge(config)
        self._state = InputState()
        self._cursor = CursorManager()
        self._is_active = False
        self._is_running = True
        self._exit_requested = False

    def run(self):
        if not self._bridge.connect():
            print(f"[ERROR] Cannot connect to port: {self._config.port}")
            return

        print(f"[OK] Connected to: {self._config.port}")

        try:
            while self._is_running:
                self._state.reset()
                self._sync_keyboard_state()
                self._wait_for_activation()

                if self._is_running:
                    self._enter_remote_mode()
        except KeyboardInterrupt:
            pass
        finally:
            self._bridge.disconnect()
            print("\n[INFO] Shutdown complete")

    def _wait_for_activation(self):
        toggle_key_name = self._get_toggle_key_display_name()
        print(f"\n[LOCAL] Press '{toggle_key_name}' to enter remote mode")

        with keyboard.Listener(on_press=self._handle_activation_key) as listener:
            listener.join()

    def _get_toggle_key_display_name(self):
        if self._config.toggle_use_e1_prefix and self._config.toggle_scan_code == 0x1D:
            return "Pause"
        if self._config.toggle_scan_code == 0x46:
            return "Scroll Lock"
        return f"Scan 0x{self._config.toggle_scan_code:02X}"

    def _handle_activation_key(self, key):
        if self._config.toggle_use_e1_prefix and key == keyboard.Key.pause:
            return False
        if (
            not self._config.toggle_use_e1_prefix
            and self._config.toggle_scan_code == 0x46
            and key == keyboard.Key.scroll_lock
        ):
            return False
        return None

    def _enter_remote_mode(self):
        self._is_active = True
        self._exit_requested = False
        toggle_key_name = self._get_toggle_key_display_name()

        print(f"\n[REMOTE] Control active ('{toggle_key_name}' = exit)")

        raw_mouse = RawMouseCapture(self._handle_raw_mouse_move)
        raw_keyboard = RawKeyboardCapture(self._handle_raw_keyboard_event)
        raw_mouse.start()
        raw_keyboard.start()

        with self._cursor.locked_context():
            mouse_listener = mouse.Listener(
                on_click=self._handle_mouse_click,
                on_scroll=self._handle_mouse_scroll,
                suppress=True,
            )
            mouse_listener.start()

            while not self._exit_requested and self._is_running:
                time.sleep(0.01)

            mouse_listener.stop()

        raw_keyboard.stop()
        raw_mouse.stop()
        self._is_active = False

    def _handle_raw_mouse_move(self, delta_x, delta_y):
        if not self._is_active:
            return

        sensitivity = self._config.sensitivity
        max_delta = self._config.max_delta

        self._state.accumulated_dx += self._clamp(
            delta_x * sensitivity, -max_delta, max_delta
        )
        self._state.accumulated_dy += self._clamp(
            delta_y * sensitivity, -max_delta, max_delta
        )
        self._flush_mouse_movement()

    def _handle_raw_keyboard_event(self, scan_code, flags, is_pressed):
        if not self._is_active:
            return

        is_extended = (flags & KeyboardFlag.E0) != 0
        is_e1 = (flags & KeyboardFlag.E1) != 0

        if is_pressed and scan_code == self._config.toggle_scan_code:
            if self._config.toggle_use_e1_prefix == is_e1:
                self._exit_requested = True
                return

        if is_extended:
            modifier = SCANCODE_TO_MODIFIER_EXTENDED.get(scan_code)
            if modifier is not None:
                self._update_modifier(modifier, is_pressed)
                return
            hid_code = SCANCODE_TO_HID_EXTENDED.get(scan_code)
        else:
            modifier = SCANCODE_TO_MODIFIER.get(scan_code)
            if modifier is not None:
                self._update_modifier(modifier, is_pressed)
                return
            hid_code = SCANCODE_TO_HID.get(scan_code)

        if hid_code:
            if is_pressed:
                self._state.pressed_keys.add(hid_code)
            else:
                self._state.pressed_keys.discard(hid_code)
            self._sync_keyboard_state()

    def _update_modifier(self, modifier, is_pressed):
        if is_pressed:
            self._state.active_modifiers |= modifier
        else:
            self._state.active_modifiers &= ~modifier
        self._sync_keyboard_state()

    def _handle_mouse_click(self, x, y, button, is_pressed):
        button_mask = PYNPUT_TO_MOUSE_BUTTON.get(button, 0)

        if is_pressed:
            self._state.mouse_buttons |= button_mask
        else:
            self._state.mouse_buttons &= ~button_mask

        self._send_mouse_report(0, 0, 0)

    def _handle_mouse_scroll(self, x, y, dx, dy):
        self._state.accumulated_wheel += dy
        self._flush_mouse_movement()

    def _sync_keyboard_state(self):
        success = self._bridge.send_keyboard_report(
            self._state.active_modifiers,
            list(self._state.pressed_keys),
        )
        if not success:
            self._handle_connection_lost()

    def _send_mouse_report(self, dx, dy, dw):
        success = self._bridge.send_mouse_report(
            self._state.mouse_buttons, dx, dy, dw
        )
        if not success:
            self._handle_connection_lost()

    def _flush_mouse_movement(self):
        now = time.monotonic()
        min_interval = 1.0 / self._config.poll_rate_hz

        if (now - self._state.last_transmission_time) < min_interval:
            return

        dx = int(self._state.accumulated_dx)
        dy = int(self._state.accumulated_dy)
        dw = int(self._state.accumulated_wheel)

        if not (dx or dy or dw):
            return

        send_dx = int(self._clamp(dx, -127, 127))
        send_dy = int(self._clamp(dy, -127, 127))
        send_dw = int(self._clamp(dw, -127, 127))

        self._state.accumulated_dx -= send_dx
        self._state.accumulated_dy -= send_dy
        self._state.accumulated_wheel -= send_dw

        self._send_mouse_report(send_dx, send_dy, send_dw)
        self._state.last_transmission_time = now

    def _handle_connection_lost(self):
        if not self._is_active:
            return

        print("\n[WARNING] Connection lost, attempting to reconnect...")

        if self._bridge.connect():
            print("[OK] Reconnected successfully")
        else:
            print("[ERROR] Failed to reconnect")
            self._is_running = False

    @staticmethod
    def _clamp(value, min_val, max_val):
        return max(min_val, min(max_val, value))


def main():
    print("═" * 50)
    print("        ESP32 KVM Bridge")
    print("═" * 50)
    config = BridgeConfig()

    detected_port = SerialBridge.find_esp32_port()
    if detected_port:
        print(f"[INFO] ESP32 detected on port: {detected_port}")
        config.port = detected_port
    else:
        print("[WARNING] ESP32 port not automatically detected")

    controller = KVMController(config)
    controller.run()


if __name__ == "__main__":
    main()
