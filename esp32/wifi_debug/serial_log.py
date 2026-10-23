"""Read boot/network diagnostics. No motor commands and no STOP on exit."""
import argparse
from pathlib import Path
import sys
import time

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / ".tools"))
import serial

p = argparse.ArgumentParser()
p.add_argument("--port", default="COM10")
p.add_argument("--seconds", type=float, default=30)
p.add_argument("--reset", action="store_true", help="Reset stopped board to capture boot log")
args = p.parse_args()
port = serial.Serial(port=None, baudrate=115200, timeout=.1)
port.dtr = port.rts = False
port.port = args.port
port.open()
if args.reset:
    port.rts = True
    time.sleep(.15)
    port.rts = False
else:
    port.write(b"STATUS\n")
end = time.monotonic() + args.seconds
(HERE / "results").mkdir(exist_ok=True)
try:
    with (HERE / "results/serial.log").open("a", encoding="utf-8") as log:
        while time.monotonic() < end:
            data = port.read(max(port.in_waiting, 1)).decode(errors="replace")
            if data:
                log.write(data)
                log.flush()
                print(data, end="", flush=True)
finally:
    port.close()
