"""Normal calibration and continuous LQI start; operator present and start authorized."""
import argparse,csv,json,queue,re,struct,sys,threading,time,zlib
from datetime import datetime,timezone
from pathlib import Path
H=Path(__file__).resolve().parent
sys.path[:0]=[r'D:/code/Embedded-related/balance_car/esp32/wifi_debug/.tools',r'D:/code/Embedded-related/balance_car/esp32/wifi_debug']
import serial
from serial.tools import list_ports
from telemetry import WIRE2,FIELDS2
parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--port');parser.add_argument('--wait-ground',action='store_true',help='After calibration wait for GROUND on stdin before BALANCE');args=parser.parse_args()
ports=[p.device for p in list_ports.comports() if p.serial_number=='5A33014245']
port=args.port or (ports[0] if len(ports)==1 else None)
assert port,'Expected unique serial device 5A33014245; specify --port if necessary'
folder=H/'results'/('live_'+datetime.now().strftime('%Y%m%d_%H%M%S'));folder.mkdir(parents=True)
s=serial.Serial(port=None,baudrate=921600,timeout=.02);s.dtr=False;s.rts=False;s.port=port;s.open()
raw=(folder/'serial.bin').open('wb');log=(folder/'commands.log').open('w',encoding='utf-8',buffering=1)
events=(folder/'events.jsonl').open('w',encoding='utf-8',buffering=1);csvfile=(folder/'samples.csv').open('w',newline='',encoding='utf-8');writer=csv.writer(csvfile);writer.writerow(['received_monotonic',*FIELDS2])
buf=bytearray();lines=[];last=None;frames=0;crc_errors=0;missing=0;previous_cycle=None
meta=dict(controller='LQI',operator_present=True,calibration_and_start_authorized=True,control_period_s=.005,current_limit_A=6,normal_operation_timeout=False)
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
   if buf.startswith((b'LQI ',b'LQI_K ')):
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
    magic=buf.find(b'BCU3');nl=buf.find(b'\n')
    if nl>=0 and (magic<0 or nl<magic):
     line=bytes(buf[:nl]).decode(errors='replace').strip();del buf[:nl+1]
     lines.append(line);log.write(line+'\n')
     if line.startswith(('STATUS ','LQI ','CAL ','RUN ','DONE ','FAULT ','REJECT')):print(line,flush=True)
    elif magic>0:del buf[:magic]
    elif len(buf)>8192:del buf[:-3]
    else:break
 csvfile.flush();raw.flush()
def command(text,prefix=None,seconds=3):
 before=len(lines);s.write((text+'\n').encode());prefix=prefix or text.split()[0];end=time.monotonic()+seconds
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
 pump(5);st=status();assert st['controller']=='LQI' and st['fault']=='0',st
 assert st['running']=='0' and st['active']=='0',st
 meta['initial_status']=st;meta['design']=json.loads((H/'lqi_design.json').read_text())['design_sha256'];save();print('LQI_LIVE_SESSION '+str(folder),flush=True)
 if st['calibrated']!='1':
  deadline=time.monotonic()+20
  while int(st['imu_us'])==0:
   if time.monotonic()>deadline:raise TimeoutError('IMU startup bias calibration did not complete')
   pump(.25);st=status();assert st['fault']=='0',st
  assert 'accepted=1' in command('CAL'),'Calibration not accepted'
  deadline=time.monotonic()+20
  while time.monotonic()<deadline:
   pump(.25);st=status()
   if st['fault']!='0':raise RuntimeError(command('FAULT'))
   if st['calibrated']=='1' and st['running']=='0':break
  else:raise TimeoutError('Normal motor calibration did not finish')
 assert st['dir_a']=='1' and st['dir_b']=='1',('Calibration directions differ from tested hardware',st)
 selected_path=H/'selected_lqi.json'
 if selected_path.exists():
  selected=json.loads(selected_path.read_text(encoding='utf-8'))
  assert selected['base_design_sha256']==meta['design'],'Saved LQI uses a different base matrix'
  target=selected['scale'];assert len(target)==8 and all(.5<=v<=1.5 for v in target)
  command('UARTSTREAM 200','UARTSTREAM');pump(.15)
  assert last and last['mode']==0 and last['fault']==0,'Expected idle LQI telemetry before reload'
  slots=FIELDS2[-10:-2];scales=[last[k] for k in slots]
  while max(abs(a-b) for a,b in zip(scales,target))>2e-6:
   step=[round(a+max(-.04,min(.04,b-a)),6) for a,b in zip(scales,target)]
   assert 'accepted=1' in command('LQISET '+' '.join(format(v,'.6g') for v in step))
   deadline=time.monotonic()+1
   while time.monotonic()<deadline:
    pump(.03)
    if max(abs(last[k]-v) for k,v in zip(slots,step))<2e-6:break
   else:raise TimeoutError('Saved LQI scales were not observed in telemetry')
   scales=[last[k] for k in slots]
  meta['reloaded_lqi_scales']=scales
  print('SAVED_LQI_RELOADED '+str(scales),flush=True)
  notch=selected.get('common_notch_mix',0.0)
  assert 0<=notch<=1
  if notch:
   assert selected.get('common_notch_frequency_Hz')==12.5,'Saved filter does not match firmware'
   observed=last['position_kp']
   while abs(observed-notch)>2e-6:
    step=round(observed+max(-.1,min(.1,notch-observed)),6)
    assert 'accepted=1' in command('LQINOTCH '+str(step))
    deadline=time.monotonic()+1
    while time.monotonic()<deadline:
     pump(.03)
     if abs(last['position_kp']-step)<2e-6:break
    else:raise TimeoutError('Saved notch mix was not observed in telemetry')
    observed=last['position_kp']
   meta['reloaded_common_notch_mix']=observed
   print('SAVED_NOTCH_RELOADED '+str(observed),flush=True)
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
  st=status();assert st['fault']=='0',st
  upright=abs(float(st['pitch']))<.174533 and abs(float(st['roll']))<.174533
  still=max(abs(float(st['raw_a'])),abs(float(st['raw_b'])))<1
  stable=stable+1 if upright and still else 0
  if stable>=3:break
  pump(.3)
 else:raise TimeoutError('Calibrated but pose never entered upright stationary start range; no BALANCE sent')
 meta['start_status']=st;assert 'accepted=1' in command('BALANCE 0 6.0','RUN')
 command('UARTSTREAM 200','UARTSTREAM');pump(.3);st=status()
 assert st['running']=='1' and st['ready']=='1' and st['fault']=='0',st
 meta['running_status']=st;save();print('LQI_CONTINUOUS_RUNNING logging enabled; no timed torque cutoff',flush=True)
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
 save();s.close();raw.close();log.close();events.close();csvfile.close()
