import argparse, csv, pathlib, time
import serial

PORT = '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A33014245-if00'
p = argparse.ArgumentParser()
p.add_argument('action', choices=['scan', 'baseline', 'verify', 'status', 'stop'])
p.add_argument('--out', default='results')
p.add_argument('--map')
p.add_argument('--gains', default='20 10 0.04')
p.add_argument('--controller', choices=['RUN','TRACK'], default='RUN')
p.add_argument('--motor', type=int, choices=[0,1])
p.add_argument('--speed', type=float, nargs='+')
p.add_argument('--label')
p.add_argument('--friction', type=float, default=.065)
p.add_argument('--velocity', default='0.04 0.3')
a = p.parse_args()
out = pathlib.Path(a.out); out.mkdir(parents=True, exist_ok=True)
s = serial.Serial(); s.port = PORT; s.baudrate = 115200; s.timeout = .15
s.dtr = False; s.rts = False; s.open()
time.sleep(1.2)
s.reset_input_buffer()
log = open(out / (a.action + '.log'), 'w', buffering=1)
def send(cmd):
    s.write((cmd+'\n').encode()); s.flush()
def line():
    b = s.readline().decode(errors='replace').strip()
    if b: log.write(b+'\n')
    return b
def until(prefix, timeout):
    end = time.monotonic()+timeout
    while time.monotonic() < end:
        v = line()
        if v.startswith(('ABORT', 'SCAN_FAIL', 'ERROR_COMMAND')):
            raise RuntimeError(v)
        if v.startswith(prefix): return v
    raise TimeoutError(prefix)
def status():
    send('STATUS'); return until('STATUS ', 3)
if a.action == 'stop':
    send('STOP'); print(until('STOPPED',3), flush=True)
    s.close(); log.close(); raise SystemExit(0)
try:
    for _ in range(30):
        time.sleep(.5)
        v = status()
        if 'ready=1 fault=0' in v: break
        if 'fault=0' not in v: raise RuntimeError(v)
    else: raise RuntimeError('not ready: '+v)
    print(v, flush=True)
    if a.action == 'status': pass
    elif a.action == 'scan':
        send('GAINS '+a.gains); until('GAINS_OK',3)
        for motor in range(2):
            send(f'SCAN {motor} 0.25 6'); until('SCAN_BEGIN',3)
            rows=[]; end=time.monotonic()+1200
            while time.monotonic()<end:
                v=line()
                if v.startswith('C,'):
                    rows.append(v.split(',')[1:])
                    if len(rows)%60==0: print(f'motor={motor} points={len(rows)}/720',flush=True)
                elif v=='SCAN_END': break
                elif v.startswith(('SCAN_FAIL','ABORT')): raise RuntimeError(v)
            if len(rows)!=720: raise RuntimeError(f'incomplete motor {motor}: {len(rows)}')
            with open(out/f'scan_m{motor}.csv','w',newline='') as f:
                w=csv.writer(f); w.writerow(['motor','step','direction','angle','speed','iq','target','id','samples','error']); w.writerows(rows)
            print(f'SCAN_SAVED motor={motor}',flush=True)
    else:
        enable = int(a.action=='verify')
        if enable and a.map:
            send('IDLE'); until('IDLE',3)
            with open(a.map) as f:
                for row in csv.DictReader(f):
                    send(f"MAP {row['motor']} {row['index']} {row['iq']}")
                    if until('MAP ',2)!='MAP 1': raise RuntimeError('map rejected')
            print('MAP_LOADED',flush=True)
        send('GAINS '+a.gains); until('GAINS_OK',3)
        send(f'FRICTION {a.friction}'); until('FRICTION_OK',3)
        send('VELOCITY '+a.velocity); until('VELOCITY_OK',3)
        for motor in ([a.motor] if a.motor is not None else range(2)):
            for speed in (a.speed if a.speed is not None else [.5,-.5]):
                send(f'{a.controller} {motor} {speed} 19 6 {enable}'); until('RUN_BEGIN',3)
                print(f'RUN motor={motor} speed={speed} ff={enable}',flush=True)
                until('RUN_END',25)
                send('DUMP'); until('DATA_BEGIN',3)
                rows=[]
                while True:
                    v=line()
                    if v=='DATA_END': break
                    if v.startswith('D,'): rows.append(v.split(',')[1:])
                    if v.startswith('ABORT'): raise RuntimeError(v)
                name=f"{a.label or a.action}_m{motor}_{'pos' if speed>0 else 'neg'}.csv"
                with open(out/name,'w',newline='') as f:
                    w=csv.writer(f); w.writerow(['t','angle','speed','iq','target','id']); w.writerows(rows)
                print(f'SAVED {name} rows={len(rows)}',flush=True)
        print(status(),flush=True)
finally:
    if a.action != 'stop': send('IDLE')
    s.close(); log.close()







