"""Read-only LAN monitor: localhost UDP telemetry in, 100 Hz SSE out."""
import collections
import http.server
import json
import pathlib
import socket
import sys
import threading
import time

ROOT = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'lqi_firmware'))
from telemetry import (FIELDS2, WIRE2, TORQUE_FIELDS, TORQUE_WIRE,
                       YAW_FIELDS, YAW_WIRE, ATTITUDE_FIELDS, ATTITUDE_WIRE,
                       IMU_FIELDS, IMU_WIRE)

FORMATS = {b'BCU3': ('control', WIRE2, FIELDS2),
           b'TOR1': ('torque', TORQUE_WIRE, TORQUE_FIELDS),
           b'YAW3': ('yaw', YAW_WIRE, YAW_FIELDS),
           b'ATT2': ('attitude', ATTITUDE_WIRE, ATTITUDE_FIELDS),
           b'IMU2': ('imu', IMU_WIRE, IMU_FIELDS)}
lock = threading.Lock()
latest = {}
received = {}
arrivals = collections.deque()

def receive():
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.bind(('127.0.0.1', 9870))
    while True:
        packet = udp.recv(2048)
        name, wire, fields = FORMATS[packet[:4]]
        values = wire.unpack(packet[:-4] if name == 'control' else packet)
        now = time.monotonic()
        with lock:
            latest[name] = dict(zip(fields, values))
            received[name] = now
            if name == 'control':
                arrivals.append(now)
                while arrivals and now - arrivals[0] > 1:
                    arrivals.popleft()

def snapshot():
    now = time.monotonic()
    with lock:
        data = dict(latest)
        data['link'] = {'age_ms': (now - received['control']) * 1000 if 'control' in received else None,
                        'source_hz': sum(t > now - 1 for t in arrivals),
                        'group_age_ms': {k: (now-v)*1000 for k,v in received.items()},
                        'server_s': now, 'push_hz': 100}
    return data

class Handler(http.server.SimpleHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(ROOT / 'static'), **kwargs)

    def log_message(self, *args):
        pass

    def translate_path(self, path):
        path = path.partition('?')[0]
        if path in ('/meshes/body.stl', '/meshes/left_wheel.stl', '/meshes/right_wheel.stl'):
            return str(ROOT.parents[1] / '模型/仿真模型/通用URDF/meshes' / path.rsplit('/', 1)[1])
        return super().translate_path(path)

    def do_GET(self):
        if self.path == '/api/state':
            body = json.dumps(snapshot(), separators=(',', ':'), allow_nan=False).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == '/events':
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Accel-Buffering', 'no')
            self.end_headers()
            self.connection.settimeout(2)
            deadline = time.monotonic()
            try:
                while True:
                    body = json.dumps(snapshot(), separators=(',', ':'), allow_nan=False)
                    self.wfile.write(('data:' + body + '\n\n').encode())
                    self.wfile.flush()
                    deadline += .01
                    now = time.monotonic()
                    if deadline < now: deadline = now
                    time.sleep(max(0, deadline-now))
            except (BrokenPipeError, ConnectionResetError, TimeoutError):
                self.close_connection = True
        else:
            super().do_GET()

if __name__ == '__main__':
    threading.Thread(target=receive, daemon=True).start()
    print('Telemetry monitor http://0.0.0.0:8766 | 100 Hz | read only', flush=True)
    http.server.ThreadingHTTPServer(('0.0.0.0', 8766), Handler).serve_forever()
