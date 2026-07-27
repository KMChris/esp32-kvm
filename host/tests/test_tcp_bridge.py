import socket

from kvm_bridge.tcp_bridge import TcpPacketSink


def test_tcp_packet_sink_sends_packet_to_configured_endpoint():
    receiver = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    receiver.bind(("127.0.0.1", 0))
    receiver.listen(1)
    receiver.settimeout(1)
    try:
        sink = TcpPacketSink("127.0.0.1", receiver.getsockname()[1])
        sink.open()
        connection, _sender = receiver.accept()
        try:
            sink.send(b"kvm-packet")
            assert connection.recv(64) == b"kvm-packet"
        finally:
            connection.close()
    finally:
        receiver.close()
        sink.close()
