"""Attach-only LQI logger. Never calibrates, starts, stops or changes gains."""
import csv,json,queue,re,struct,sys,threading,time,zlib
from datetime import datetime,timezone
from pathlib import Path
H=Path(__file__).resolve().parent
sys.path[:0]=[r'D:/code/Embedded-related/balance_car/esp32/wifi_debug/.tools',r'D:/code/Embedded-related/balance_car/esp32/wifi_debug']
import serial
from telemetry import WIRE2,FIELDS2
folder=H/'results'/('live_'+datetime.now().strftime('%Y%m%d_%H%M%S'));folder.mkdir(parents=True)
s=serial.Serial(port=None,baudrate=921600,timeout=.02);s.dtr=False;s.rts=False;s.port='COM10';s.open()
raw=(folder/'serial.bin').open('wb');log=(folder/'commands.log').open('w',encoding='utf-8',buffering=1)
events=(folder/'events.jsonl').open('w',encoding='utf-8',buffering=1);csvfile=(folder/'samples.csv').open('w',newline='',encoding='utf-8');writer=csv.writer(csvfile);writer.writerow(['received_monotonic',*FIELDS2])
buf=bytearray();lines=[];last=None;frames=0;crc_errors=0;missing=0;previous_cycle=None
meta=dict(controller='LQI',operator_present=True,calibration_and_start_authorized=False,attach_only=True,control_period_s=.005,current_limit_A=2,normal_operation_timeout=False)
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
try:
 pump(.15);st=status();assert st['controller']=='LQI' and st['fault']=='0',st
 assert st['running']=='1' and st['active']=='2' and st['ready']=='1',st
 meta['initial_status']=st;meta['design']=json.loads((H/'lqi_design.json').read_text())['design_sha256'];save();print('LQI_LIVE_SESSION '+str(folder),flush=True)
 meta['attached_running_status']=st;save()
 command('UARTSTREAM 200','UARTSTREAM');pump(.3);st=status()
 assert st['running']=='1' and st['ready']=='1' and st['fault']=='0',st
 meta['running_status']=st;save();print('LQI_CONTINUOUS_RUNNING logging enabled; no timed torque cutoff',flush=True)
 commands=queue.Queue()
 def inputs():
  for line in sys.stdin:commands.put(line.strip())
 threading.Thread(target=inputs,daemon=True).start();due=time.monotonic()+5;reported_fault=False
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
