"""Forward validated telemetry to the local read-only web monitor."""
import socket

_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
_socket.setblocking(False)

def forward(packet):
    try:
        _socket.sendto(packet, ('127.0.0.1', 9870))
    except OSError:
        pass  # Monitoring must never interrupt serial capture or fault retrieval.
