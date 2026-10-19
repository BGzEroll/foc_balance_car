"""Bounded bench command. STOP is confirmed before config/run and before DUMP."""
from pathlib import Path
import argparse,json,time,serial
p=argparse.ArgumentParser();p.add_argument('label');p.add_argument('--command')
p.add_argument('--mask',type=int,default=1,choices=[1,2,3]);p.add_argument('--zero',nargs=3,type=float,action='append',default=[])
p.add_argument('--limit',nargs=2,type=float,default=[.015,.12]);p.add_argument('--sense',nargs=2,type=float,action='append',default=[])
p.add_argument('--gains',nargs=3,type=float,default=[.060318579,226.194672,0]);p.add_argument('--velocity',nargs=3,type=float);p.add_argument('--reset',action='store_true');p.add_argument('--warmup',action='store_true');a=p.parse_args();root=Path(__file__).resolve().parent
s=serial.Serial();s.port='/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A33014245-if00';s.baudrate=115200;s.timeout=.08;s.dtr=False;s.rts=False
data=bytearray();s.open()
def read_for(seconds):
 until=time.monotonic()+seconds
 while time.monotonic()<until:data.extend(s.read(65536))
def send(x):s.write((x+'\n').encode());s.flush()
def parse(prefix):return [json.loads(l[len(prefix):]) for l in data.decode(errors='replace').splitlines() if l.startswith(prefix)]
def stopped():
 send('STOP');read_for(.25);send('STATUS');read_for(.25);st=parse('STATUS ')
 if not st or st[-1]['running'] or st[-1]['requested']:raise RuntimeError('No confirmed stopped state')
 return st[-1]
def setting(command,ack):
 before=len(data);send(command);read_for(.2)
 if ack.encode() not in data[before:]:raise RuntimeError('Setting rejected: '+command)
try:
 if a.reset:s.rts=True;time.sleep(.1);s.rts=False
 read_for(3 if a.reset else 1.5);st=stopped()
 if st.get('version') not in [13,14,15,16,17,18]:raise RuntimeError('Unexpected firmware version')
 # The existing IMU zero-rate calibration takes 1000 stationary samples at
 # 200 Hz. Keep both motors stopped until it has published a fresh result.
 if a.command or a.warmup:
  warmup_end=time.monotonic()+10
  while True:
   h=parse('HEALTH ')
   if h and h[-1]['imu_age_us']<50000:break
   if time.monotonic()>warmup_end:raise RuntimeError('IMU stationary warmup did not complete')
   send('STATUS');read_for(.3)
  st=stopped()
 setting('MOTOR '+str(a.mask),'MOTOR_OK')
 for motor,direction,zero in a.zero:setting(f'ZERO {int(motor)} {direction} {zero:.9f}','ZERO_OK')
 for motor,gain in a.sense:setting(f'SENSE {int(motor)} {gain}','SENSE_OK')
 setting('LIMIT '+' '.join(map(str,a.limit)),'LIMIT_OK')
 setting('GAINS '+' '.join(map(str,a.gains)),'GAINS_OK')
 if a.velocity:setting('VEL '+' '.join(map(str,a.velocity)),'VEL_OK')
 if a.command:
  for motor,agekey in [(1,'left_age_us'),(2,'right_age_us')]:
   if a.mask&motor and st[agekey]>=1500:raise RuntimeError('Encoder is stale')
  before=len(data);send(a.command);read_for(int(a.command.split()[2])/1000+.35)
  if b'ACCEPTED' not in data[before:]:raise RuntimeError('Run rejected')
  stopped();send('DUMP');end=time.monotonic()+100
  while time.monotonic()<end:
   data.extend(s.read(65536))
   if b'\nEND\r\n' in data or b'\nEND\n' in data:break
  else:raise RuntimeError('Incomplete dump')
  stopped()
finally:
 send('STOP');read_for(.2);s.close();(root/(a.label+'.log')).write_bytes(data)
text=data.decode(errors='replace');meta=parse('META ');setup=parse('SETUP ');metric=parse('METRIC ');rows=[];lut={};header=None
for line in text.splitlines():
 if line.startswith('CSV '):header=line[4:]
 elif line.startswith('R '):rows.append(line[2:])
 elif line.startswith('LUT '):x=line[4:].split(',');lut[x[0]]=list(map(int,x[1:]))
if a.command:
 if not meta or len(rows)!=meta[-1]['rows'] or not rows:raise RuntimeError('No complete experiment data')
 (root/(a.label+'.csv')).write_text(header+'\n'+'\n'.join(rows)+'\n')
 (root/(a.label+'.json')).write_text(json.dumps({'meta':meta[-1],'setup':setup[-1],'metric':metric,'latency':parse('LATENCY '),'lut_mv':lut,'health':parse('HEALTH '),'status':parse('STATUS ')},indent=2))
else:
 (root/(a.label+'.json')).write_text(json.dumps({'health':parse('HEALTH '),'status':parse('STATUS ')},indent=2))
print(json.dumps({'label':a.label,'meta':meta,'setup':setup,'metric':metric,'status':parse('STATUS ')[-1:]},ensure_ascii=False))
if any(x in text for x in ['Guru Meditation','assert failed','panic','abort()']) or (meta and meta[-1]['reason']!=1):raise SystemExit(1)
