"""Record raw IMU on a fixed stand; optional short, speed-bounded unloaded probes."""
import argparse,csv,json,re,struct,time,zlib
from datetime import datetime
from pathlib import Path
import serial
from telemetry import FIELDS2,WIRE2

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--seconds',type=float,default=180)
p.add_argument('--motor-probes',action='store_true')
p.add_argument('--attitude',action='store_true',help='Fixed stand: learn bias for 30s, then free heading; motors remain stopped')
a=p.parse_args()
assert not(a.attitude and a.motor_probes),'Attitude rest calibration is a separate stopped-stand experiment'
out=Path(__file__).resolve().parent/'results/imu'/datetime.now().strftime('%Y%m%d_%H%M%S')
out.mkdir(parents=True)
imu_wire=struct.Struct('<IIQQI7hH6fI')
s=serial.Serial(port=None,baudrate=921600,timeout=.02)
s.dtr=False;s.rts=False;s.port='/dev/ttyACM0';s.open()
raw=(out/'serial.bin').open('wb');imu_file=(out/'imu.csv').open('w',newline='')
tele_file=(out/'telemetry.csv').open('w',newline='');log=(out/'commands.log').open('w',buffering=1)
att_wire=struct.Struct('<IIQ22f7IfI');att_file=(out/'attitude.csv').open('w',newline='');aw=csv.writer(att_file)
aw.writerow(['stage','sequence','time_us','qw','qx','qy','qz','bx','by','bz','roll','pitch','yaw','legacy_roll','legacy_pitch','legacy_yaw','gyro_x','gyro_y','gyro_z','p_rx','p_ry','p_rz','p_bx','p_by','p_bz','compute_us','gravity_updates','gravity_rejected','rest_updates','selected','confirmed_rest','stack_high_water','nis'])
iw=csv.writer(imu_file);tw=csv.writer(tele_file)
iw.writerow(['stage','sequence','time_us','ax','ay','az','temperature_raw','gx','gy','gz','gyro_x','gyro_y','gyro_z','pitch','roll','yaw','read_began_us','native_sequence'])
tw.writerow(['stage',*FIELDS2])
buf=bytearray();lines=[];latest=None;stage='boot';bad=0;nimu=0;last_sequence=None;missing=0

def pump(seconds):
 global buf,latest,bad,nimu,last_sequence,missing
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  b=s.read(max(1,s.in_waiting));raw.write(b);buf.extend(b)
  while buf:
   size=72 if buf.startswith(b'IMU2') else 224 if buf.startswith(b'BCU3') else 140 if buf.startswith(b'ATT1') else 0
   if size:
    if len(buf)<size:break
    packet=bytes(buf[:size]);del buf[:size]
    if zlib.crc32(packet[:-4])&0xffffffff!=struct.unpack_from('<I',packet,size-4)[0]:bad+=1;continue
    if size==72:
     v=imu_wire.unpack(packet);iw.writerow([stage,v[1],v[2],*v[5:12],*v[13:19],v[3],v[4]]);nimu+=1
     if last_sequence is not None:missing+=(v[1]-last_sequence-1)&0xffffffff
     last_sequence=v[1]
    elif size==140:
     v=att_wire.unpack(packet);aw.writerow([stage,*v[1:-1]])
    else:
     v=WIRE2.unpack(packet[:-4]);latest=dict(zip(FIELDS2,v));tw.writerow([stage,*v])
     if stage.startswith('probe') and (latest['fault'] or max(abs(latest['raw_a']),abs(latest['raw_b']))>6):
      s.write(b'STOP\n');stage_change='probe_speed_stop';log.write(stage_change+'\n')
    continue
   positions=[x for x in (buf.find(b'IMU2'),buf.find(b'BCU3'),buf.find(b'ATT1')) if x>=0]
   magic=min(positions) if positions else -1;nl=buf.find(b'\n')
   if nl>=0 and (magic<0 or nl<magic):
    text=bytes(buf[:nl]).decode(errors='replace').strip();del buf[:nl+1]
    lines.append(text);log.write(text+'\n')
   elif magic>0:del buf[:magic]
   elif len(buf)>8192:del buf[:-3]
   else:break
 imu_file.flush();tele_file.flush();att_file.flush();raw.flush()

def command(text,prefix=None):
 before=len(lines);s.write((text+'\n').encode());end=time.monotonic()+3
 while time.monotonic()<end:
  pump(.03)
  for line in lines[before:]:
   if line.startswith(('REJECT','BUSY')):raise RuntimeError(line)
   if line.startswith(prefix or text.split()[0]):return line
 raise TimeoutError(text)

def status():return dict(re.findall(r'(\w+)=([^ ]+)',command('STATUS')))

try:
 pump(6);command('STOP','STOPPED');st=status();assert st['running']=='0' and st['fault']=='0',st
 info=command('IMUINFO');print(info,flush=True)
 assert 'who_am_i=0x68 ' in info and 'sample_divider=0 ' in info and 'dlpf=3 ' in info and 'gyro_range=0x08 ' in info and 'accel_range=0x00 ' in info,info
 command('UARTSTREAM 0' if a.attitude else 'UARTSTREAM 200');command('IMUSTREAM 200')
 if a.attitude:
  command('ATTSTREAM 200');command('ATTREST 1','ATTITUDE')
 print('IMU_RECORDING '+str(out),flush=True)
 stage='rest_training' if a.attitude else 'static';began=time.monotonic();deadline=began+a.seconds;next_progress=began+30
 while time.monotonic()<deadline:
  pump(min(5,max(0,deadline-time.monotonic())))
  if a.attitude:
   st=status();assert st['running']=='0' and st['fault']=='0',st
   if stage=='rest_training' and time.monotonic()-began>=30:
    command('ATTREST 0','ATTITUDE');stage='static';print('REST_CONSTRAINT_OFF; free-heading validation',flush=True)
  else:assert latest and latest['mode']==0 and latest['fault']==0,latest
  if time.monotonic()>=next_progress:
   print('STATIC samples='+str(nimu),flush=True);next_progress+=30
 if a.motor_probes:
  stage='calibration';assert 'accepted=1' in command('CAL')
  deadline=time.monotonic()+25
  while time.monotonic()<deadline:
   pump(.3);st=status();assert st['fault']=='0',st
   if st['calibrated']=='1' and st['running']=='0':break
  else:raise TimeoutError('Motor calibration')
  stage='post_cal_static';pump(15)
  for motor in (0,1):
   for current in (.08,-.08):
    stage='probe_'+str(motor)+'_'+str(current)
    print(stage,flush=True);command(f'PROBE {motor} {current} 400','RUN');pump(.6)
    command('STOP','STOPPED');stage='coast_'+str(motor)+'_'+str(current);pump(8)
    st=status();assert st['fault']=='0',st
  stage='final_static';pump(20)
finally:
 s.write(b'STOP\nATTREST 0\nATTSTREAM 0\nIMUSTREAM 0\nUARTSTREAM 0\n');pump(.3)
 final=status();s.close()
 report={'folder':str(out),'imu_frames':nimu,'crc_errors':bad,'missing_stream_sequences':missing,'register_readback':info,'image_sha256':json.loads((out.parents[1]/'flashed_identity.json').read_text())['image_sha256'],'attitude_experiment':a.attitude,'confirmed_rest_training_s':30 if a.attitude else 0,'final':final,'scope':'Fixed stand, no ground balance or true imposed angular-motion reference. Rest constraint, if requested, uses operator-confirmed fixed stand only and is disabled after training. Motor probes, if requested, are hardware-timed 400ms at 0.08A with host stop above 6rad/s.'}
 (out/'capture.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
