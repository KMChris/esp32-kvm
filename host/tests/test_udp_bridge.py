import socket

from kvm_bridge.udp_bridge import UdpPacketSink


def test_udp_packet_sink_sends_packet_to_configured_endpoint():
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(("127.0.0.1", 0))
    receiver.settimeout(1)
    try:
        sink = UdpPacketSink("127.0.0.1", receiver.getsockname()[1])
        sink.open()
        sink.send(b"kvm-packet")

        payload, _sender = receiver.recvfrom(64)
        assert payload == b"kvm-packet"
    finally:
        receiver.close()
        sink.close()
