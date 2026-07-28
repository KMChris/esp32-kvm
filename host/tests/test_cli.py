import importlib.util
from pathlib import Path


_MAIN_PATH = Path(__file__).resolve().parents[1] / "main.py"
_SPEC = importlib.util.spec_from_file_location("host_main", _MAIN_PATH)
assert _SPEC is not None and _SPEC.loader is not None
_MODULE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)
parse_args = _MODULE.parse_args
key_translation = _MODULE.key_translation
PynputCapture = _MODULE.PynputCapture
restart_argv = _MODULE.restart_argv


def test_parse_args_selects_udp_transport_and_default_esp32_endpoint():
    args = parse_args(["--transport", "udp"])

    assert args.transport == "udp"
    assert args.udp_host == "192.168.4.1"
    assert args.udp_port == 3333


def test_parse_args_rejects_out_of_range_udp_port():
    for value in ("0", "65536"):
        try:
            parse_args(["--transport", "udp", "--udp-port", value])
        except SystemExit as exc:
            assert exc.code == 2
        else:
            raise AssertionError(f"expected UDP port {value} to be rejected")


def test_restart_preserves_the_active_bridge_command_line():
    assert restart_argv("/usr/bin/python3", ["host/main.py", "--port", "/dev/cu.usbserial-0001"]) == [
        "/usr/bin/python3", "host/main.py", "--port", "/dev/cu.usbserial-0001"
    ]


def test_key_translation_uses_macos_virtual_key_when_keydown_has_no_character():
    key = type("Key", (), {"char": None, "name": "", "vk": 39})()
    tilde_key_up = type("Key", (), {"char": "˜", "name": "", "vk": 50})()

    assert key_translation(key) == (0x34, 0)
    assert key_translation(tilde_key_up) == (0x35, 0)


def test_remote_toggle_waits_for_chord_release_before_stopping_keyboard_capture():
    class Controller:
        active = True

        def toggle_remote_mode(self):
            self.active = not self.active

        def key_down(self, _usage, _modifier):
            pass

        def key_up(self, _usage, _modifier):
            pass

    key = lambda name="", char=None: type("Key", (), {"name": name, "char": char, "vk": None})()
    controller = Controller()
    capture = PynputCapture(controller, "ctrl+alt+cmd+g")

    for name in ("ctrl", "alt", "cmd"):
        assert capture._key_press(key(name=name)) is None
    assert capture._key_press(key(char="g")) is None
    assert controller.active is False
    assert capture._stop_after_return is True
    assert capture._key_release(key(char="g")) is False
