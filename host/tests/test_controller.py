from kvm_bridge.controller import RemoteInputController
from kvm_bridge.protocol import encode_keyboard_report, encode_mouse_report


class RecordingSink:
    def __init__(self):
        self.packets = []

    def send(self, packet):
        self.packets.append(packet)


def test_controller_sends_reports_only_while_remote_mode_is_active():
    sink = RecordingSink()
    controller = RemoteInputController(sink)

    controller.key_down(0x04)
    assert sink.packets == []

    assert controller.toggle_remote_mode() is True
    controller.key_down(0x04)
    controller.key_up(0x04)
    controller.mouse_move(12, -3)

    assert sink.packets == [
        encode_keyboard_report(0, [0x04]),
        encode_keyboard_report(0, []),
        encode_mouse_report(0, 12, -3, 0),
    ]


def test_controller_releases_pressed_input_when_leaving_remote_mode():
    sink = RecordingSink()
    controller = RemoteInputController(sink)

    controller.toggle_remote_mode()
    controller.key_down(0x04)
    assert controller.toggle_remote_mode() is False

    assert sink.packets[-2:] == [
        encode_keyboard_report(0, []),
        encode_mouse_report(0, 0, 0, 0),
    ]
