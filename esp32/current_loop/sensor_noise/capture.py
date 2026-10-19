"""Finite raw/processed sensor bursts. PWM remains stopped except explicit finite speed cases."""
from pathlib import Path
import csv,json,time,serial,argparse
p=argparse.ArgumentParser();p.add_argument('--stationary',type=int,default=8);p.add_argument('--moving',action='store_true');a=p.parse_args();root=Path(__file__).resolve().parent
s=serial.Serial();s.port='/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A33014245-if00';s.baudrate=115200;s.timeout=.05;s.dtr=False;s.rts=False;data=bytearray();results=[];s.open()
def read_for(sec):
 end=time.monotonic()+sec
 while time.monotonic()<end:data.extend(s.read(65536))
def send(x):s.write((x+'\n').encode());s.flush()
def parse(prefix,start=0):return [json.loads(l[len(prefix):]) for l in data[start:].decode(errors='replace').splitlines() if l.startswith(prefix)]
def stopped():
 send('STOP');read_for(.15);send('STATUS');read_for(.2);st=parse('STATUS ')
 if not st or st[-1]['running'] or st[-1]['requested']:raise RuntimeError('No confirmed STOP')
 return st[-1]
def setting(x,ack):
 n=len(data);send(x);read_for(.12)
 if ack.encode() not in data[n:]:raise RuntimeError('Rejected '+x)
def burst(label,speed=None):
 before=len(data);st=stopped();setting('SRECSTART','SREC_START')
 if speed is not None:
  n=len(data);send(f'SPEED {speed} 4000 25');read_for(4.3)
  if b'ACCEPTED' not in data[n:]:raise RuntimeError('Speed rejected')
  st=stopped()
  if st['reason']!=1:raise RuntimeError('Motor protection fired')
 else:read_for(3.3);st=stopped()
 send('SRECDUMP');end=time.monotonic()+35
 while time.monotonic()<end:
  data.extend(s.read(65536))
  if b'SREC_END' in data[before:]:break
 else:raise RuntimeError('Incomplete sensor export')
 block=data[before:].decode(errors='replace');meta=parse('SREC_META ',before)
 if not meta or meta[-1]['count']!=600:raise RuntimeError('Incomplete burst')
 rows=[l[7:] for l in block.splitlines() if l.startswith('SREC_R ')];header=next(l[9:] for l in block.splitlines() if l.startswith('SREC_CSV '))
 if len(rows)!=600:raise RuntimeError('Row mismatch')
 (root/(label+'.csv')).write_text(header+'\n'+'\n'.join(rows)+'\n');(root/(label+'.log')).write_text(block)
 result={'label':label,'speed_rad_s':speed,'meta':meta[-1],'status':st,'health':parse('HEALTH ',before)}
 (root/(label+'.json')).write_text(json.dumps(result,indent=2));results.append(result);print(json.dumps({'label':label,'samples':len(rows),'speed':speed,'status':st}),flush=True)
try:
 read_for(2);st=stopped()
 if st['version']!=24:raise RuntimeError('Recorder firmware required')
 deadline=time.monotonic()+12
 while True:
  send('STATUS');read_for(.2);h=parse('HEALTH ')
  if h and h[-1]['imu_age_us']<50000:break
  if time.monotonic()>deadline:raise RuntimeError('IMU warmup incomplete')
 for k in range(a.stationary):burst(f'still_{k+1:02d}')
 if a.moving:
  for x,ack in [('MOTOR 3','MOTOR_OK'),('ZERO 1 -1 5.330672807','ZERO_OK'),('ZERO 2 1 2.164932114','ZERO_OK'),('GAINS .060318579 226.194672 0','GAINS_OK'),('LIMIT .04 .4','LIMIT_OK'),('VEL .2 .4 .7','VEL_OK')]:setting(x,ack)
  for w in [10,20]:burst(f'moving_{w}',w)
 stopped()
finally:
 send('STOP');read_for(.2);s.close();(root/'capture_session.log').write_bytes(data);(root/'capture_summary.json').write_text(json.dumps(results,indent=2))
print('CAPTURE_COMPLETE_STOPPED',flush=True)
