"""Capture one bounded ground trial through the existing continuous logger."""
import argparse,csv,json,math,os,pathlib,re,time
p=argparse.ArgumentParser(description=__doc__);p.add_argument('source',type=pathlib.Path);p.add_argument('kind',choices=['linear','yaw']);p.add_argument('amplitude',type=float);p.add_argument('label');p.add_argument('--raw-imu',action='store_true');p.add_argument('--move-seconds',type=float,default=3);a=p.parse_args()
assert .05<=a.amplitude<=.20 if a.kind=='linear' else 15<=a.amplitude<=55
out=pathlib.Path(__file__).resolve().parent/'results/identification'/a.label;out.mkdir(parents=True,exist_ok=False)
log=(a.source/'commands.log').open();log.seek(0,2)
def send(text):
 fd=os.open('/dev/ttyACM0',os.O_RDWR|os.O_NOCTTY|os.O_NONBLOCK)
 try:os.write(fd,(text+'\n').encode())
 finally:os.close(fd)
def command(text):
 log.seek(0,2);send(text)
 deadline=time.monotonic()+3;pending=''
 while time.monotonic()<deadline:
  pending+=log.read()
  for line in pending.splitlines():
   if line.startswith(('REJECT','BUSY')):raise RuntimeError(line)
   if line.startswith(text.split()[0]+' '):return line
  time.sleep(.02)
 raise TimeoutError(text)
files={};writers={};handles=[]
for name in ['samples.csv','imu.csv','yaw.csv']:
 f=(a.source/name).open();header=f.readline().strip().split(',');f.seek(0,2)
 dest=(out/name).open('w',newline='');w=csv.writer(dest);w.writerow(header)
 files[name]=(f,header);writers[name]=w;handles.extend([f,dest])
rows=[]
def capture(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  for name,(f,header) in files.items():
   while True:
    pos=f.tell();line=f.readline()
    if not line:break
    if not line.endswith('\n'):f.seek(pos);break
    values=next(csv.reader([line]));writers[name].writerow(values)
    if name=='samples.csv':rows.append(dict(zip(header,map(float,values))))
  time.sleep(.02)
try:
 st=command('STATUS');assert 'running=1 ' in st and 'fault=0 ' in st,st
 capture(2);assert rows and abs(rows[-1]['speed'])<.02 and abs(rows[-1]['distance'])<.015,rows[-1] if rows else 'no data'
 begin=rows[-1]['sample_us'];text='DYNTEST '+str(a.amplitude) if a.kind=='linear' else 'YAWTEST '+str(a.amplitude)+' '+str(a.move_seconds);reply=command(text);assert 'accepted=1' in reply,reply
 if a.raw_imu:send('IMUSTREAM 200')
 capture(18 if a.kind=='linear' else 4*(a.move_seconds+2)+2)
 if a.raw_imu:send('IMUSTREAM 0');time.sleep(.3)
 end=command('DYNEND' if a.kind=='linear' else 'YAWEND');assert 'accepted=1' in end,end
 capture(5);st=command('STATUS');assert 'fault=0 ' in st and 'running=1 ' in st,st
 active=[r for r in rows if r['sample_us']>=begin]
 r0=active[0];theta0=r0['yaw'];x=y=0.;peak=0;prev=r0
 for r in active[1:]:
  ds=.0325*((r['angle_b']-prev['angle_b'])-(r['angle_a']-prev['angle_a']))/2+.0325*(r['pitch']-prev['pitch'])
  heading=(r['yaw']+prev['yaw'])/2-theta0;x+=ds*math.cos(heading);y+=ds*math.sin(heading);peak=max(peak,math.hypot(x,y));prev=r
 report=dict(kind=a.kind,amplitude=a.amplitude,move_seconds=a.move_seconds,start_us=begin,end_us=active[-1]['sample_us'],source=str(a.source),command_reply=reply,end_reply=end,status=st,rows=len(active),peak_distance_m=max(abs(r['distance']-r0['distance']) for r in active),peak_planar_encoder_odometry_m=peak,peak_pitch_deg=max(abs(r['pitch'])*180/math.pi for r in active),peak_current_command_A=max(max(abs(r['command_a']),abs(r['command_b'])) for r in active),scope='Encoder/IMU odometry only; no independent ground position or heading measurement.')
 (out/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
finally:
 if a.raw_imu:send('IMUSTREAM 0')
 for f in handles:f.close()
