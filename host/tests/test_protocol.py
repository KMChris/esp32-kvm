from kvm_bridge.protocol import encode_keyboard_report, encode_mouse_report


def test_keyboard_packet_matches_upstream_uart_protocol():
    packet = encode_keyboard_report(0x02, [0x04, 0x05])

    assert packet == bytes([0xFD, 0x02, 0x00, 0x04, 0x05, 0, 0, 0, 0, 0x03])


def test_mouse_packet_matches_upstream_uart_protocol():
    packet = encode_mouse_report(0x05, -2, 300, -1)

    assert packet == bytes([0xFD, 0x00, 0x03, 0x05, 0xFE, 0xFF, 0x2C, 0x01, 0xFF, 0xD5])
