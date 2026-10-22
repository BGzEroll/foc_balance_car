"""Remote serial capture; opening retains DTR/RTS low. Optional bounded commands."""
import argparse, time
from pathlib import Path
import serial
p=argparse.ArgumentParser()
p.add_argument('label'); p.add_argument('--seconds',type=float,default=12)
p.add_argument('--reset',action='store_true'); p.add_argument('--command')
a=p.parse_args()
s=serial.Serial(); s.port='/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A33014245-if00'
s.baudrate=115200; s.timeout=.03; s.dtr=False; s.rts=False
data=bytearray(); s.open()
try:
    if a.reset:
        s.rts=True; time.sleep(.12); s.rts=False
    if a.command:
        warm=time.monotonic()+2
        while time.monotonic()<warm: data.extend(s.read(65536))
        s.write((a.command+'\n').encode()); s.flush()
    end=time.monotonic()+a.seconds
    while time.monotonic()<end: data.extend(s.read(65536))
finally:
    s.write(b'STOP\n'); s.flush(); time.sleep(.1); data.extend(s.read(65536)); s.close()
out=Path(__file__).resolve().parent/(a.label+'.log'); out.write_bytes(data)
print(data.decode(errors='replace')[-18000:])
