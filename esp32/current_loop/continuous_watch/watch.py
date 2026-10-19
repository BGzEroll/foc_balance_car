"""Explicitly authorized continuous suspended-motor demo; never auto-restarts.

MCU expires output within 2s without KEEP. SIGTERM/STOP file sends STOP.
The WATCH lease test is mandatory before the persistent 10rad/s run.
"""
from pathlib import Path
import argparse, json, os, signal, time
import serial

root=Path(__file__).resolve().parent
ap=argparse.ArgumentParser();ap.add_argument('--probe',action='store_true');args=ap.parse_args()
s=serial.Serial();s.port='/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A33014245-if00';s.baudrate=115200;s.timeout=.03;s.dtr=False;s.rts=False
pending=bytearray();latest={};raw=open(root/'serial.log','ab',buffering=0)
def pump(seconds):
 end=time.monotonic()+seconds;lines=[]
 while time.monotonic()<end:
  b=s.read(4096)
  if b:raw.write(b);pending.extend(b)
  while b'\n' in pending:
   line,_,rest=pending.partition(b'\n');pending[:]=rest;line=line.decode(errors='replace').strip();lines.append(line)
   for prefix in ['STATUS ','HEALTH ']:
    if line.startswith(prefix):latest[prefix.strip()]=json.loads(line[len(prefix):])
   if any(x in line for x in ['Guru Meditation','panic','assert failed','abort()']):raise RuntimeError(line)
 return lines
def send(command):s.write((command+'\n').encode());s.flush()
def request(command,ack,timeout=.35):
 send(command);lines=pump(timeout)
 if not any(line.startswith(ack) for line in lines):raise RuntimeError('No ACK: '+command+' '+repr(lines))
def status():
 latest.pop('STATUS',None);request('STATUS','STATUS ');return latest['STATUS'].copy()
def stop():
 send('STOP');pump(.15);st=status()
 if st['running'] or st['requested']:raise RuntimeError('STOP not confirmed')
 return st
def shutdown(*_):raise SystemExit('operator stop')
signal.signal(signal.SIGTERM,shutdown);signal.signal(signal.SIGINT,shutdown)
opened=False
try:
 s.open();opened=True;pump(1.5);st=stop();print('INITIAL_STATUS',json.dumps(st),flush=True)
 if args.probe:raise SystemExit(0)
 assert st['version']==19,'WATCH firmware required'
 end=time.monotonic()+12
 while latest.get('HEALTH',{}).get('imu_age_us',10**9)>=50000:
  if time.monotonic()>end:raise RuntimeError('IMU stationary warmup failed')
  status()
 assert 5<st['bus_V']<13 and max(st['left_age_us'],st['right_age_us'])<1500,'Bus/encoder not ready'
 for command,ack in [('MOTOR 3','MOTOR_OK'),('ZERO 1 -1 5.330672807','ZERO_OK'),('ZERO 2 1 2.164932114','ZERO_OK'),('GAINS .060318579 226.194672 0','GAINS_OK'),('LIMIT .04 .4','LIMIT_OK'),('VEL .2 .4 .7','VEL_OK')]:request(command,ack)
 # Prove loss of host heartbeat disarms output and KEEP cannot re-arm it.
 request('WATCH 3 2000 25','ACCEPTED');pump(2.1);st=status()
 assert not st['running'] and not st['requested'] and st['reason']==1,'Lease expiry failed'
 request('KEEP','ERROR keep');st=stop();print('LEASE_EXPIRY_AND_NO_REARM_PASS',json.dumps(st),flush=True)
 request('WATCH 10 2000 25','ACCEPTED',.15)
 (root/'watch.pid').write_text(str(os.getpid()))
 started=time.monotonic();last_status=0;previous=None
 while not (root/'STOP').exists():
  request('KEEP','KEEP_OK',.12)
  now=time.monotonic()
  if now-last_status>=1:
   st=status();now=time.monotonic()
   if not st['running'] or not st['requested'] or st['reason']!=0:raise RuntimeError('MCU stopped: '+json.dumps(st))
   if not 5<st['bus_V']<13 or max(st['left_age_us'],st['right_age_us'])>=1500:raise RuntimeError('Unhealthy bus/encoder')
   report={'elapsed_s':now-started,'status':st,'speed_rad_s':None,'pid':os.getpid(),'state':'running','target_rad_s':10,'current_limit_A':.7,'base_current_pi':True,'lease_ms':2000}
   if previous:
    before,prior=previous;dt=now-before;speed=[-(st['left']-prior['left'])/dt,(st['right']-prior['right'])/dt];report['speed_rad_s']=speed
    if max(map(abs,speed))>20:raise RuntimeError('Host measured overspeed: '+repr(speed))
    if now-started>4 and max(abs(x-10) for x in speed)>3:raise RuntimeError('Speed tracking not maintained: '+repr(speed))
   previous=(now,st);last_status=now
   (root/'live.json').write_text(json.dumps(report,indent=2));print('WATCH_RUNNING',json.dumps(report),flush=True)
  pump(.08)
 print('STOP_FILE_REQUEST',flush=True)
finally:
 if opened:
  try:st=stop();(root/'final_stopped.json').write_text(json.dumps(st,indent=2));print('STOPPED',json.dumps(st),flush=True)
  finally:s.close()
 raw.close()
