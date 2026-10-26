"""Normal calibration and continuous PID start; operator present and start authorized."""
import argparse,csv,json,queue,re,struct,sys,threading,time,zlib
from datetime import datetime,timezone
from pathlib import Path
H=Path(__file__).resolve().parent
sys.path[:0]=[r'D:/code/Embedded-related/balance_car/esp32/wifi_debug/.tools',r'D:/code/Embedded-related/balance_car/esp32/wifi_debug']
import serial
from serial.tools import list_ports
from telemetry import WIRE2,FIELDS2,ATTITUDE_WIRE,ATTITUDE_FIELDS,YAW_WIRE,YAW_FIELDS,IMU_WIRE,IMU_FIELDS,TORQUE_WIRE,TORQUE_FIELDS
parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--port');parser.add_argument('--wait-cal',action='store_true');parser.add_argument('--wait-ground',action='store_true',help='After calibration wait for GROUND on stdin before BALANCE');args=parser.parse_args()
ports=[p.device for p in list_ports.comports() if p.serial_number=='5A33014245']
port=args.port or (ports[0] if len(ports)==1 else None)
assert port,'Expected unique serial device 5A33014245; specify --port if necessary'
folder=H/'results'/('live_'+datetime.now().strftime('%Y%m%d_%H%M%S'));folder.mkdir(parents=True)
s=serial.Serial(port=None,baudrate=921600,timeout=.02);s.dtr=False;s.rts=False;s.port=port;s.open()
raw=(folder/'serial.bin').open('wb');log=(folder/'commands.log').open('w',encoding='utf-8',buffering=1)
events=(folder/'events.jsonl').open('w',encoding='utf-8',buffering=1);csvfile=(folder/'samples.csv').open('w',newline='',encoding='utf-8');writer=csv.writer(csvfile);writer.writerow(['received_monotonic',*FIELDS2])
attfile=(folder/'attitude.csv').open('w',newline='',encoding='utf-8');attwriter=csv.writer(attfile);attwriter.writerow(ATTITUDE_FIELDS)
yawfile=(folder/'yaw.csv').open('w',newline='',encoding='utf-8');yawwriter=csv.writer(yawfile);yawwriter.writerow(YAW_FIELDS)
imufile=(folder/'imu.csv').open('w',newline='',encoding='utf-8');imuwriter=csv.writer(imufile);imuwriter.writerow(IMU_FIELDS)
torquefile=(folder/'torque.csv').open('w',newline='',encoding='utf-8');torquewriter=csv.writer(torquefile);torquewriter.writerow(TORQUE_FIELDS)
buf=bytearray();lines=[];last=None;frames=0;crc_errors=0;missing=0;previous_cycle=None
meta=dict(controller='PID',operator_present=True,calibration_and_start_authorized=True,control_period_s=.005,current_limit_A=6,normal_operation_timeout=False)
def save():
 meta.update(frames=frames,crc_errors=crc_errors,missing_cycles=missing,latest=last)
 (folder/'metadata.json').write_text(json.dumps(meta,indent=2),encoding='utf-8')
def pump(seconds):
 global last,frames,crc_errors,missing,previous_cycle
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  b=s.read(max(1,s.in_waiting))
  if b:raw.write(b);buf.extend(b)
  while buf:
   if buf.startswith(b'TOR1'):
    size=TORQUE_WIRE.size
    if len(buf)<size:break
    packet=bytes(buf[:size]);del buf[:size]
    if zlib.crc32(packet[:-4])&0xffffffff!=struct.unpack_from('<I',packet,size-4)[0]:crc_errors+=1;continue
    torquewriter.writerow(TORQUE_WIRE.unpack(packet));continue
   if buf.startswith(b'IMU2'):
    size=IMU_WIRE.size
    if len(buf)<size:break
    packet=bytes(buf[:size]);del buf[:size]
    if zlib.crc32(packet[:-4])&0xffffffff!=struct.unpack_from('<I',packet,size-4)[0]:crc_errors+=1;continue
    imuwriter.writerow(IMU_WIRE.unpack(packet));continue
   if buf.startswith((b'ATT2',b'YAW3')):
    is_att=buf.startswith(b'ATT2');size=124 if is_att else 68
    if len(buf)<size:break
    packet=bytes(buf[:size]);del buf[:size]
    if zlib.crc32(packet[:-4])&0xffffffff!=struct.unpack_from('<I',packet,size-4)[0]:crc_errors+=1;continue
    if is_att:attwriter.writerow(ATTITUDE_WIRE.unpack(packet))
    else:yawwriter.writerow(YAW_WIRE.unpack(packet))
    continue
   if buf.startswith(b'ENC1'):
    if len(buf)<24:break
    magic,count,tau_us,enabled,first_us=struct.unpack_from('<4I Q',buf)
    assert count==2048,('Unexpected encoder capture size',count)
    size=24+count*16+4
    if len(buf)<size:break
    packet=bytes(buf[:size]);del buf[:size]
    if zlib.crc32(packet[:-4])&0xffffffff!=struct.unpack_from('<I',packet,size-4)[0]:
     crc_errors+=1;print('ENC_CAPTURE_CRC_ERROR',flush=True);continue
    capture=folder/('encoder_'+str(first_us)+'.bin');capture.write_bytes(packet)
    print('ENC_CAPTURE '+str(capture)+' tau_us='+str(tau_us)+' enabled='+str(enabled),flush=True);continue
   if buf.startswith((b'PID ',b'PID_K ')):
    nl=buf.find(b'\n')
    if nl<0:break
    line=bytes(buf[:nl]).decode(errors='replace').strip();del buf[:nl+1]
    lines.append(line);log.write(line+'\n');print(line,flush=True);continue
   if buf.startswith(b'BCU3'):
    if len(buf)<224:break
    packet=bytes(buf[:224]);del buf[:224]
    if zlib.crc32(packet[:220])&0xffffffff!=struct.unpack_from('<I',packet,220)[0]:crc_errors+=1;continue
    values=WIRE2.unpack(packet[:220]);last=dict(zip(FIELDS2,values));writer.writerow([time.perf_counter(),*values]);frames+=1
    cycle=last['cycle']
    if previous_cycle is not None and cycle>previous_cycle:missing+=max(0,cycle-previous_cycle-1)
    previous_cycle=cycle
   else:
    positions=[v for v in [buf.find(b'BCU3'),buf.find(b'ENC1'),buf.find(b'ATT2'),buf.find(b'YAW3'),buf.find(b'IMU2'),buf.find(b'TOR1')] if v>=0];magic=min(positions) if positions else -1;nl=buf.find(b'\n')
    if nl>=0 and (magic<0 or nl<magic):
     line=bytes(buf[:nl]).decode(errors='replace').strip();del buf[:nl+1]
     lines.append(line);log.write(line+'\n')
     if line.startswith(('STATUS ','PID ','CAL ','RUN ','DONE ','FAULT ','REJECT','ENC')):print(line,flush=True)
    elif magic>0:del buf[:magic]
    elif len(buf)>8192:del buf[:-3]
    else:break
 csvfile.flush();attfile.flush();yawfile.flush();imufile.flush();torquefile.flush();raw.flush()
def command(text,prefix=None,seconds=3):
 before=len(lines);s.write((text+'\n').encode());prefix=prefix or ('RUN' if text.split()[0] in ('SHADOW','BALANCE','PROBE') else text.split()[0]);end=time.monotonic()+seconds
 while time.monotonic()<end:
  pump(.03)
  for line in lines[before:]:
   if line.startswith(('REJECT','BUSY')):raise RuntimeError(line)
   if line.startswith(prefix):
    events.write(json.dumps(dict(utc=datetime.now(timezone.utc).isoformat(),command=text,reply=line))+'\n');return line
 raise TimeoutError(text)
def status():
 line=command('STATUS');return dict(re.findall(r'(\w+)=([^ ]+)',line))

commands=queue.Queue()
def inputs():
 for line in sys.stdin:commands.put(line.strip())
threading.Thread(target=inputs,daemon=True).start()
try:
 pump(5);st=status();assert st['controller']=='PID' and st['fault']=='0',st
 assert st['running']=='0' and st['active']=='0',st
 meta['initial_status']=st;meta['design']=json.loads((H/'pid_design.json').read_text())['design_sha256'];save();print('PID_LIVE_SESSION '+str(folder),flush=True)
 if st['calibrated']!='1':
  deadline=time.monotonic()+20
  while int(st['imu_us'])==0:
   if time.monotonic()>deadline:raise TimeoutError('IMU startup bias calibration did not complete')
   pump(.25);st=status();assert st['fault']=='0',st
  if args.wait_cal:
   command('UARTSTREAM 200');print('WAITING_FOR_CAL motors off; encoder capture available; send CAL when wheels are free',flush=True)
   waiting=True
   while waiting:
    pump(.05)
    while not commands.empty():
     text=commands.get()
     if text=='QUIT':raise SystemExit
     if text=='CAL':waiting=False;break
     if text:command(text)
  assert 'accepted=1' in command('CAL'),'Calibration not accepted'
  deadline=time.monotonic()+20
  while time.monotonic()<deadline:
   pump(.25);st=status()
   if st['fault']!='0':raise RuntimeError(command('FAULT'))
   if st['calibrated']=='1' and st['running']=='0':break
  else:raise TimeoutError('Normal motor calibration did not finish')
 assert st['dir_a']=='1' and st['dir_b']=='1',('Calibration directions differ from tested hardware',st)
 selected=json.loads((H/'selected_pid.json').read_text(encoding='utf-8'))
 assert selected['design_sha256']==meta['design']
 target=selected['gains'];assert len(target)==10
 command('UARTSTREAM 200');pump(.15)
 assert last and last['mode']==0 and last['fault']==0
 assert 'accepted=1' in command('PIDSET '+' '.join(format(v,'.10g') for v in target))
 deadline=time.monotonic()+1
 while time.monotonic()<deadline:
  pump(.03)
  if max(abs(last[k]-v) for k,v in zip(FIELDS2[-10:],target))<2e-6:break
 else:raise TimeoutError('PID gains not observed in telemetry')
 meta['reloaded_pid_gains']=target
 meta['encoder_ab_us']=4000
 meta['calibration_status']=st;save();print('CALIBRATION_COMPLETE waiting for upright stationary pose',flush=True)
 if args.wait_ground:
  print('WAITING_FOR_GROUND no BALANCE sent; send GROUND after wheels touch floor',flush=True)
  grounded=False;due=time.monotonic()+5
  while not grounded:
   pump(.05)
   while not commands.empty():
    text=commands.get()
    if text=='QUIT':raise SystemExit
    if text=='GROUND':grounded=True
    elif text:command(text)
   if time.monotonic()>=due:status();save();due=time.monotonic()+5
 deadline=time.monotonic()+120;stable=0
 while time.monotonic()<deadline:
  aligned='accepted=1' in command('ATTALIGN');pump(.05)
  st=status();assert st['fault']=='0',st
  upright=abs(float(st['pitch']))<.174533 and abs(float(st['roll']))<.174533
  still=max(abs(float(st['raw_a'])),abs(float(st['raw_b'])))<1
  stable=stable+1 if upright and still and aligned else 0
  if stable>=3:break
  pump(.3)
 else:raise TimeoutError('Calibrated but pose never entered upright stationary start range; no BALANCE sent')
 assert 'accepted=1' in command('ATTALIGN');pump(.1)
 pose=command('ATTITUDE');match=re.search(r'tilt_error_rad=([^ ]+)',pose);assert match and float(match.group(1))<.035,pose
 meta['start_attitude']=pose
 assert 'mode=zero_rate_bias' in command('YAW')
 meta['yaw_bias_correction_enabled']=True
 meta['start_status']=st;assert 'accepted=1' in command('BALANCE 0 6.0','RUN')
 command('UARTSTREAM 200','UARTSTREAM');pump(.3);st=status()
 assert st['running']=='1' and st['ready']=='1' and st['fault']=='0',st
 meta['running_status']=st;save();print('PID_CONTINUOUS_RUNNING logging enabled; no timed torque cutoff',flush=True)
 due=time.monotonic()+5;reported_fault=False
 while True:
  pump(.05)
  while not commands.empty():
   text=commands.get()
   if text=='QUIT':raise SystemExit
   if text:command(text)
  if last and (last['fault']!=0 or last['mode']==0) and not reported_fault:
   reported_fault=True;print('CONTROLLER_STOPPED '+str(last['fault']),flush=True);command('FAULT');save()
  if time.monotonic()>=due:
   status();save();print(json.dumps(dict(frames=frames,crc_errors=crc_errors,missing_cycles=missing)),flush=True);due=time.monotonic()+10
finally:
 save();s.close();raw.close();log.close();events.close();csvfile.close();attfile.close();yawfile.close();imufile.close();torquefile.close()
