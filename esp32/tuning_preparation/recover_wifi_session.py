"""Explicit recovery: normal OTA reboot, normal CAL, then passive continuous logger.

Never clears a latched fault in place or restores reconstructed alignment.
BALANCE must be sent explicitly through the interactive logger after CAL succeeds.
"""
import argparse
import json
import runpy
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from ota.client import OTA
from wifi_debug.client import Client

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--host', default='192.168.114.243')
p.add_argument('--output', type=Path, required=True)
p.add_argument('--no-update', action='store_true', help='Use the firmware already running after a normal power reset')
p.add_argument('--apply', type=Path, help='Current-unit PID profile to apply in the passive logger')
args = p.parse_args()
if not args.no_update:
    result = OTA(args.host).update(HERE.parent/'wifi_debug/firmware/balance_car.bin')
    (HERE/'results/phase_2p5A_ota.json').write_text(json.dumps(result, indent=2))
c = Client(args.host)
try:
    print(c.command('STREAM2 UDP 0', 'STREAM'), flush=True)
    print(c.command('CAL'), flush=True)
    deadline = time.monotonic()+12
    while time.monotonic() < deadline:
        time.sleep(.15)
        status = c.command('STATUS')
        print(status, flush=True)
        if 'fault=0' not in status:
            raise RuntimeError(c.command('FAULT'))
        if 'calibrated=1' in status and 'running=0' in status:
            break
    else:
        raise TimeoutError('Normal calibration did not complete')
finally:
    c.close()
sys.argv = ['session.py', args.host, '--seconds', '7200', '--interactive',
            '--usb', 'no', '--output', str(args.output)]
if args.apply:
    sys.argv += ['--apply', str(args.apply)]
runpy.run_path(str(HERE/'session.py'), run_name='__main__')
