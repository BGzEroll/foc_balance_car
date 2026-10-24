import argparse,csv,pathlib,signal,time
import serial
p=argparse.ArgumentParser()
p.add_argument('--speed',type=float,default=.1)
p.add_argument('--out',default='../matlab/anticogging_results')
a=p.parse_args()
root=pathlib.Path(a.out); root.mkdir(parents=True,exist_ok=True)
s=serial.Serial(); s.port='/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A33014245-if00'
s.baudrate=115200; s.timeout=1; s.dtr=False; s.rts=False; s.open()
time.sleep(1.2); s.reset_input_buffer()
def send(cmd): s.write((cmd+'\n').encode()); s.flush()
def until(prefix,timeout=4):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        line=s.readline().decode(errors='replace').strip()
        if line.startswith(prefix): return line
        if line.startswith('ABORT'): raise RuntimeError(line)
    raise TimeoutError(prefix)
def interrupt(*_): raise KeyboardInterrupt
signal.signal(signal.SIGTERM,interrupt)
f=open(root/'demo_live.csv','w',newline='',buffering=1)
w=csv.writer(f); w.writerow(['t','angle0','speed0','iq0','angle1','speed1','iq1','ready','fault'])
try:
    for _ in range(30):
        send('STATUS'); state=until('STATUS ')
        if 'ready=1 fault=0' in state: break
        if 'fault=0' not in state: raise RuntimeError(state)
        time.sleep(.5)
    else: raise RuntimeError('Not ready')
    print(state,flush=True)
    send('GAINS 20 10 0.04'); until('GAINS_OK')
    send('FRICTION .065'); until('FRICTION_OK')
    send(f'DEMO {a.speed}'); print(until('DEMO_BEGIN'),flush=True)
    history=[]
    while True:
        line=until('LIVE,',5)
        row=line.split(',')[1:]; w.writerow(row)
        if row[-2:]!=['1','0']: raise RuntimeError(line)
        history.append([float(x) for x in row])
        if len(history)%10==0:
            x,y=history[-10],history[-1]; dt=y[0]-x[0]
            print('DEMO_LIVE samples=%d mean_speed0=%.5f mean_speed1=%.5f ready=%s fault=%s' %
                  (len(history),(y[1]-x[1])/dt,(y[4]-x[4])/dt,row[-2],row[-1]),flush=True)
except KeyboardInterrupt:
    print('Stopping demo',flush=True)
finally:
    send('STOP')
    try: print(until('STOPPED'),flush=True)
    finally: s.close(); f.close()
