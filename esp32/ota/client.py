"""Authenticated LAN OTA. Never starts/calibrates motors. Python standard library only."""
import argparse
import hashlib
import http.client
import json
from pathlib import Path
import socket
import struct
import sys
import time

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / 'esp32'))
from wifi_debug.client import Client as DebugClient


def stopped(host, check_stream=False):
    c = DebugClient(host)
    try:
        line = c.command('STATUS')
        if 'running=0' not in line or 'ready=0' not in line or 'active=0' not in line:
            raise RuntimeError('OTA requires stopped hardware; finish the control session first')
        assert c.command('GAINS').startswith('GAINS schema=current_pid_v1 ')
        if check_stream:
            c.command(f'STREAM2 UDP 200 {c.udp.getsockname()[1]}', 'STREAM')
            end = time.monotonic() + .3
            while time.monotonic() < end:
                c.pump(.02)
            if len(c.records) < 10:
                raise RuntimeError('UDP telemetry health probe failed')
        return line
    finally:
        c.close()


def image_info(data):
    if len(data) < 1024 or data[0] != 0xE9 or struct.unpack_from('<H', data, 12)[0] != 9:
        raise ValueError('Expected an ESP32-S3 application image')
    if struct.unpack_from('<I', data, 32)[0] != 0xABCD5432:
        raise ValueError('Application descriptor missing (do not upload a bootloader/merged image)')
    version = data[48:80].split(b'\0')[0].decode('ascii')
    project = data[80:112].split(b'\0')[0].decode('ascii')
    if project != 'balance_car' or not version.startswith('current_pid_v1-'):
        raise ValueError('Application project/schema mismatch')
    return dict(bytes=len(data), sha256=hashlib.sha256(data).hexdigest(),
                elf_sha256=data[176:208].hex(), version=version, project=project)


class OTA:
    def __init__(self, host, token_file=HERE / 'token.json'):
        self.host = host
        self.token = json.loads(Path(token_file).read_text())['token']
        if len(self.token) != 64:
            raise ValueError('Invalid OTA token')

    def request(self, path, body=None, headers=None, method=None):
        connection = http.client.HTTPConnection(self.host, 3335, timeout=15)
        try:
            all_headers = {'X-OTA-Token': self.token, 'Connection': 'close'}
            all_headers.update(headers or {})
            connection.request(method or ('GET' if path == 'status' else 'POST'),
                               '/ota/' + path, body=body, headers=all_headers)
            response = connection.getresponse()
            data = json.loads(response.read())
            if response.status != 200:
                raise RuntimeError(f'HTTP {response.status}: {data}')
            return data
        finally:
            connection.close()

    def stage(self, image):
        stopped(self.host)
        data = Path(image).read_bytes()
        info = image_info(data)
        status = self.request('status')
        if status['pending_verify'] or status['locked'] or status['staged']:
            raise RuntimeError('Device already locked; confirm pending boot or cancel staged image')
        began = time.monotonic()
        response = self.request('upload', data,
                                {'X-Image-SHA256': info['sha256'], 'X-Image-Schema': 'current_pid_v1',
                                 'Content-Type': 'application/octet-stream'})
        status = self.request('status')
        assert status['staged'] and status['locked'] and status['motor_ack']
        assert status['staged_sha256'] == info['sha256']
        return dict(image=info, seconds=round(time.monotonic()-began, 3), response=response, staged_status=status)

    def wait_boot(self, expected_elf, previous_partition, confirm=True):
        # Fresh connections only; never retry the write/commit operation automatically.
        end = time.monotonic() + 50
        last = None
        while time.monotonic() < end:
            try:
                status = self.request('status')
                if (status['running'] != previous_partition and status['pending_verify'] and
                        status['healthy'] and status['motor_ack']):
                    if status['elf_sha256'] != expected_elf:
                        raise ValueError('Booted image identity mismatch; leave locked for rollback')
                    stopped(self.host, check_stream=True)  # Proves TCP, UDP and current PID schema work.
                    if confirm:
                        self.request('confirm')
                        status = self.request('status')
                        assert not status['pending_verify'] and not status['locked'] and status['state'] == 2
                    return status
            except (OSError, http.client.HTTPException, RuntimeError) as exc:
                last = str(exc)
            time.sleep(1)
        raise TimeoutError(f'New image not confirmed; rollback remains armed. Last error: {last}')

    def update(self, image):
        result = self.stage(image)
        self.request('reboot')
        result['boot_status'] = self.wait_boot(result['image']['elf_sha256'], result['staged_status']['running'])
        return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('action', choices=['status', 'stage', 'reboot', 'confirm', 'cancel', 'update'])
    p.add_argument('--host', default='192.168.114.243')
    p.add_argument('--token-file', type=Path, default=HERE / 'token.json')
    p.add_argument('--image', type=Path, default=ROOT / 'esp32/wifi_debug/firmware/balance_car.bin')
    args = p.parse_args()
    ota = OTA(args.host, args.token_file)
    result = (getattr(ota, args.action)(args.image) if args.action in ('stage', 'update')
              else ota.request(args.action))
    print(json.dumps(result, indent=2))
    if args.action == 'update':
        folder = HERE / 'results'; folder.mkdir(exist_ok=True)
        (folder / 'last_update.json').write_text(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
