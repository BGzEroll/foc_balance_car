"""Finite stand tests over SSH. Output is always stopped when the client exits."""
import argparse
import base64
import json
from pathlib import Path
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument("action", choices=["status", "cal", "run", "live"])
parser.add_argument("--command", default="SHADOW 1000 0.25")
parser.add_argument("--name", default="status")
parser.add_argument("--calibrate", action="store_true")
args = parser.parse_args()
program = r'''
import json,os,select,serial,sys,time
sys.excepthook=lambda kind,error,trace: print('BENCH_ERROR '+str(error),file=sys.stderr)
args=json.loads(sys.argv[1])
port=serial.Serial(port=None,baudrate=115200,timeout=.05)
port.dtr=False;port.rts=False
port.port='/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A33014245-if00'
port.open()
def send(command):
    port.write((command+'\n').encode());port.flush()
def collect(seconds,until=None):
    end=time.monotonic()+seconds;data=bytearray()
    while time.monotonic()<end:
        chunk=port.read(4096)
        data.extend(chunk)
        if until and until in data:break
    text=data.decode(errors='replace');print(text,end='',flush=True)
    return text
try:
    collect(.2)
    send('STATUS');text=collect(.3)
    # Opening this CH343 port resets the ESP on this host. Stay in one session.
    for i in range(35):
        if 'imu_us=' in text and 'imu_us=0 ' not in text:break
        send('STATUS');text=collect(.2)
    else:raise RuntimeError('IMU did not initialize')
    if args['action'] in ('cal','run') or (args['action']=='live' and args.get('calibrate')):
        send('CAL');collect(.3)
        for i in range(35):
            send('STATUS');text=collect(.2)
            if 'calibrated=1' in text:break
        else:raise RuntimeError('Calibration did not finish')
    if args['action']=='run':
        send(args['command']);text=collect(3.5,b'DONE result=')
        if 'DONE result=' not in text:raise RuntimeError('No run completion')
        collect(.2)
        send('DUMP');text=collect(12,b'CSV_END')
        if 'CSV_END' not in text:raise RuntimeError('Incomplete CSV')
    elif args['action']=='live':
        print('LIVE_READY default=STOP manual_CAL',flush=True)
        with open('/home/bgzerol/embedded_project/foc_balance_car/esp32/balance_car/build/upper_validation/live.log','w',buffering=1) as log:
            input_buffer=bytearray()
            quit_requested=False
            while True:
                readers,_,_=select.select([port.fileno(),sys.stdin],[],[],.1)
                if sys.stdin in readers:
                    chunk=os.read(sys.stdin.fileno(),4096)
                    if not chunk:break
                    input_buffer.extend(chunk)
                    while b'\n' in input_buffer:
                        raw,_,input_buffer=input_buffer.partition(b'\n')
                        command=raw.decode().strip()
                        if command=='QUIT':
                            quit_requested=True
                            break
                        if command:
                            log.write('COMMAND '+command+'\n')
                            send(command)
                    if quit_requested:break
                if port.fileno() in readers:
                    text=port.read(max(1,port.in_waiting)).decode(errors='replace')
                    log.write(text);print(text,end='',flush=True)
finally:
    send('STOP');collect(.15)
    send('STATUS');collect(.2)
    port.close()
'''
encoded = base64.b64encode(program.encode()).decode()
options = base64.b64encode(json.dumps(vars(args)).encode()).decode()
command = ("/home/bgzerol/.espressif/tools/python/v6.0.2/venv/bin/python3 -c "
           "'import base64,sys;sys.argv=[\"bench\",base64.b64decode(\"" + options +
           "\").decode()];exec(base64.b64decode(\"" + encoded + "\"))'")
if args.action == "live":
    raise SystemExit(subprocess.run(["ssh", "develop", command]).returncode)
result = subprocess.run(["ssh", "develop", command], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
text = result.stdout.decode(errors="replace")
folder = Path(__file__).resolve().parent / "results"
folder.mkdir(exist_ok=True)
(folder / (args.name + ".log")).write_text(text, encoding="utf-8")
if "CSV_BEGIN" in text and "CSV_END" in text:
    csv = text.split("CSV_BEGIN", 1)[1].split("\n", 1)[1].split("CSV_END", 1)[0]
    (folder / (args.name + ".csv")).write_text(csv.replace("\r", ""), encoding="utf-8")
lines = text.splitlines()
states = [line for line in lines if line.startswith('STATUS')]
print("\n".join(line for line in lines
                if line.startswith(('CAL ','RUN ','DONE ','STOP','CSV_','BENCH_ERROR','REJECT'))))
print("\n".join(states[-3:]))
raise SystemExit(result.returncode)
