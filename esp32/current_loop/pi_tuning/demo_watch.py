"""Observed, finite v18 demonstration. No firmware changes or background restart."""
from pathlib import Path
import json,time,serial
r=Path(__file__).resolve().parent
s=serial.Serial();s.port='/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A33014245-if00';s.baudrate=115200;s.timeout=.05;s.dtr=False;s.rts=False
data=bytearray();events=[];s.open()
def read_for(sec):
 end=time.monotonic()+sec
 while time.monotonic()<end:data.extend(s.read(16384))
def send(x):s.write((x+'\n').encode());s.flush()
def parsed(prefix):return [json.loads(x[len(prefix):]) for x in data.decode(errors='replace').splitlines() if x.startswith(prefix)]
def stopped():
 send('STOP');read_for(.15);send('STATUS');read_for(.15);q=parsed('STATUS ')
 if not q or q[-1]['running'] or q[-1]['requested']:raise RuntimeError('STOP not confirmed')
 return q[-1]
def setting(x,ack):
 n=len(data);send(x);read_for(.12)
 if ack.encode() not in data[n:]:raise RuntimeError('Rejected '+x)
try:
 read_for(1.5);st=stopped()
 if st['version']!=18:raise RuntimeError('Unexpected firmware')
 end=time.monotonic()+12
 while True:
  send('STATUS');read_for(.2);h=parsed('HEALTH ')
  if h and h[-1]['imu_age_us']<50000:break
  if time.monotonic()>end:raise RuntimeError('Stationary sensor warmup failed')
 st=stopped()
 if not 5<st['bus_V']<13 or max(st['left_age_us'],st['right_age_us'])>=1500:raise RuntimeError('Unready bus/encoder')
 for cmd,ack in [('MOTOR 3','MOTOR_OK'),('ZERO 1 -1 5.330672807','ZERO_OK'),('ZERO 2 1 2.164932114','ZERO_OK'),('GAINS .060318579 226.194672 0','GAINS_OK'),('LIMIT .04 .4','LIMIT_OK'),('VEL .2 .4 .7','VEL_OK')]:setting(cmd,ack)
 for w in [5,10,-10,20]:
  print(f'DEMO_START {w} rad/s, 4 seconds',flush=True);n=len(data);send(f'SPEED {w} 4000 25');read_for(4.3)
  if b'ACCEPTED' not in data[n:]:raise RuntimeError('Run rejected')
  st=stopped();events.append({'speed_rad_s':w,'status':st});print(json.dumps(events[-1]),flush=True)
  if st['reason']!=1:raise RuntimeError('Protection stopped demonstration')
  read_for(1.5)
 setting('LIMIT .015 .12','LIMIT_OK');st=stopped();print('DEMO_COMPLETE_STOPPED',json.dumps(st),flush=True)
finally:
 send('STOP');read_for(.2);s.close();(r/'demo_watch.log').write_bytes(data);(r/'demo_watch.json').write_text(json.dumps(events,indent=2))
