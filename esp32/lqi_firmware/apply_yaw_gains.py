"""Ramp only yaw PID gains through the existing logger; preserve all balance gains."""
import argparse,csv,json,os,pathlib,time
p=argparse.ArgumentParser(description=__doc__);p.add_argument('source',type=pathlib.Path);p.add_argument('kp',type=float);p.add_argument('ki',type=float);p.add_argument('kd',type=float);a=p.parse_args()
with (a.source/'samples.csv').open() as f:
 header=f.readline().strip().split(',');f.seek(max(f.tell(),(a.source/'samples.csv').stat().st_size-32768));lines=f.readlines();last=dict(zip(header,map(float,next(csv.reader([lines[-1]])))))
assert last['mode']==2 and last['fault']==0 and abs(last['speed'])<.02,last
names=['velocity_kp','velocity_ki','pitch_kd','pitch_kp','pitch_ki','yaw_kp','yaw_ki','yaw_kd','position_kp','pitch_trim_rad'];old=[last[k] for k in names];new=old.copy();new[5:8]=[a.kp,a.ki,a.kd]
with (a.source/'commands.log').open() as log:
 for step in range(1,6):
  values=[x+(y-x)*step/5 for x,y in zip(old,new)];log.seek(0,2)
  fd=os.open('/dev/ttyACM0',os.O_RDWR|os.O_NOCTTY|os.O_NONBLOCK)
  try:os.write(fd,('PIDSET '+' '.join(format(v,'.10g' if i==9 else '.6g') for i,v in enumerate(values))+'\n').encode())
  finally:os.close(fd)
  deadline=time.monotonic()+3;reply=''
  while time.monotonic()<deadline:
   reply+=log.read()
   if 'PIDSET accepted=1' in reply:break
   if 'REJECT' in reply:raise RuntimeError(reply)
   time.sleep(.02)
  else:raise TimeoutError('PIDSET')
  time.sleep(.2)
print(json.dumps(dict(before=old,after=new)),flush=True)
