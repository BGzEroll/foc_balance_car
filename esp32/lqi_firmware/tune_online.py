"""MATLAB-screened online LQI A/B trials on develop. Keeps balancing after trials."""
import csv,json,math,queue,re,statistics,struct,sys,threading,time,zlib
from collections import deque
from datetime import datetime
from pathlib import Path
import serial
from serial.tools import list_ports
from telemetry import WIRE2,FIELDS2

H=Path(__file__).resolve().parent
plan=json.loads((H/'results/online_lqi/candidates.json').read_text())
reference_scale=plan.get('start_scale',[1.]*8)
folder=H/'results'/('tuning_'+datetime.now().strftime('%Y%m%d_%H%M%S'));folder.mkdir(parents=True)
port=next(p.device for p in list_ports.comports() if p.serial_number=='5A33014245')
s=serial.Serial(port=None,baudrate=921600,timeout=.02);s.dtr=False;s.rts=False;s.port=port;s.open()
csvfile=(folder/'samples.csv').open('w',newline='');writer=csv.writer(csvfile);writer.writerow(['stage',*FIELDS2])
events=(folder/'events.jsonl').open('w',buffering=1)
buf=bytearray();lines=[];samples=deque(maxlen=20000);stage='startup';last=None;bad_crc=0
current=[1.]*8;history=[];baseline=None;previous=None;last_print=0

def event(kind,**kw):
 item=dict(time=time.time(),kind=kind,stage=stage,**kw);events.write(json.dumps(item)+'\n')
 print(json.dumps(item),flush=True)

def pump(seconds,check=False):
 global last,bad_crc,last_print
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  chunk=s.read(max(1,s.in_waiting));buf.extend(chunk)
  while buf:
   if buf.startswith((b'LQI ',b'LQI_K ')):
    nl=buf.find(b'\n')
    if nl<0:break
    lines.append(bytes(buf[:nl]).decode(errors='replace').strip());del buf[:nl+1];continue
   if buf.startswith(b'BCU3'):
    if len(buf)<224:break
    packet=bytes(buf[:224]);del buf[:224]
    if zlib.crc32(packet[:220])&0xffffffff!=struct.unpack_from('<I',packet,220)[0]:bad_crc+=1;continue
    last=dict(zip(FIELDS2,WIRE2.unpack(packet[:220])));samples.append(last);writer.writerow([stage,*WIRE2.unpack(packet[:220])])
   else:
    magic=buf.find(b'BCU3');nl=buf.find(b'\n')
    if nl>=0 and (magic<0 or nl<magic):
     line=bytes(buf[:nl]).decode(errors='replace').strip();del buf[:nl+1];lines.append(line)
     if line.startswith(('FAULT ','DONE ','REJECT')):event('device',reply=line)
    elif magic>0:del buf[:magic]
    elif len(buf)>8192:del buf[:-3]
    else:break
  if check and last:
   if last['fault'] or last['mode']!=2 or last['flags']&6!=6:raise RuntimeError('Controller stopped '+str(last))
   if abs(last['pitch'])>.2618 or abs(last['speed'])>.35:raise ValueError('Trial tilt/speed guard')
 csvfile.flush()
 if check and time.monotonic()-last_print>10:
  event('live',pitch=last['pitch'],fault=last['fault'],revision=last['revision']);last_print=time.monotonic()

def command(text,prefix=None,timeout=4):
 start=len(lines);s.write((text+'\n').encode());end=time.monotonic()+timeout
 while time.monotonic()<end:
  pump(.03)
  for line in lines[start:]:
   if line.startswith(('REJECT','BUSY')):raise RuntimeError(line)
   if line.startswith(prefix or text.split()[0]):return line
 raise TimeoutError(text)

def status():
 reply=command('STATUS');return dict(re.findall(r'(\w+)=([^ ]+)',reply))

def ramp(target,guard=True):
 global current
 target=[round(float(v),6) for v in target]
 while max(abs(a-b) for a,b in zip(target,current))>1e-6:
  new=[round(a+max(-.04,min(.04,b-a)),6) for a,b in zip(current,target)]
  text='LQISET '+' '.join(format(v,'.6g') for v in new);assert len(text)<96
  reply=command(text);assert 'accepted=1' in reply
  revision=int(re.search(r'revision=(\d+)',reply).group(1))
  current=new;event('parameters',scale=current,reply=reply)
  deadline=time.monotonic()+1
  while (not last or last['revision']!=revision) and time.monotonic()<deadline:pump(.03,guard)
  if not last or last['revision']!=revision:raise RuntimeError('Applied revision not observed')
  pump(.3,guard)

def rms(values):
 return math.sqrt(statistics.fmean(v*v for v in values))

def measure(label,seconds=35,settle=8):
 global stage
 stage=label+'_settle';event('settling',seconds=settle);pump(settle,True)
 stage=label;start=last['sample_us'];event('measuring',seconds=seconds);pump(seconds,True)
 rows=[r for r in samples if r['sample_us']>start and r['mode']==2 and not r['fault']]
 assert len(rows)>seconds*180,'Insufficient telemetry'
 vals=lambda name:[r[name] for r in rows]
 pitch=[r['pitch']-r['target_pitch'] for r in rows]
 cmd=[v for r in rows for v in (r['command_a'],r['command_b'])]
 result=dict(label=label,samples=len(rows),scale=current.copy(),revision=rows[-1]['revision'],
  pitch_rms_deg=rms(pitch)*180/math.pi,pitch_ac_rms_deg=statistics.pstdev(pitch)*180/math.pi,
  pitch_mean_deg=statistics.fmean(pitch)*180/math.pi,pitch_peak_deg=max(map(abs,pitch))*180/math.pi,
  pitch_rate_rms=rms(vals('pitch_rate')),speed_rms=rms(vals('speed')),
  distance_std=statistics.pstdev(vals('distance')),yaw_rms_deg=rms(vals('yaw'))*180/math.pi,
  yaw_ac_rms_deg=statistics.pstdev(vals('yaw'))*180/math.pi,current_rms_A=rms(cmd),
  current_peak_A=max(map(abs,cmd)),saturation_fraction=statistics.fmean(vals('saturated')),
  bus_mean_V=statistics.fmean(vals('bus')),crc_errors=bad_crc)
 history.append(result);save();event('metrics',metrics=result);return result

def score(m,reference):
 weights=dict(pitch_rms_deg=.35,pitch_rate_rms=.2,speed_rms=.2,yaw_rms_deg=.1,current_rms_A=.15)
 return sum(w*(m[k]/max(reference[k],1e-5))**2 for k,w in weights.items())

def save(extra=None):
 record=dict(history=history,current_scale=current,plan=plan,extra=extra)
 (folder/'report.json').write_text(json.dumps(record,indent=2))

try:
 pump(12);st=status();assert st['controller']=='LQI' and st['fault']=='0',st
 if st['calibrated']!='1':
  assert 'accepted=1' in command('CAL');deadline=time.monotonic()+20
  while time.monotonic()<deadline:
   pump(.3);st=status();assert st['fault']=='0',st
   if st['calibrated']=='1' and st['running']=='0':break
  else:raise TimeoutError('Calibration did not complete')
 assert st['dir_a']=='1' and st['dir_b']=='1',st
 assert 'accepted=1' in command('LQISET 1 1 1 1 1 1 1 1')
 command('UARTSTREAM 200','UARTSTREAM');pump(.2)
 if reference_scale!=[1.]*8:ramp(reference_scale,False)
 if st['active']!='2':
  stable=0;deadline=time.monotonic()+120
  while time.monotonic()<deadline:
   st=status();assert st['fault']=='0',st
   stable=stable+1 if abs(float(st['pitch']))<.174533 and abs(float(st['roll']))<.174533 and max(abs(float(st['raw_a'])),abs(float(st['raw_b'])))<1 else 0
   if stable>=3:break
   pump(.3)
  else:raise TimeoutError('Waiting for upright stationary pose')
  assert 'accepted=1' in command('BALANCE 0 6.0','RUN')
 command('UARTSTREAM 200','UARTSTREAM');pump(.5,True)
 event('running',folder=str(folder));baseline=measure('A0_baseline');previous=baseline
 trials=[]
 for i,candidate in enumerate(plan['candidates']):
  try:
   stage='ramp_candidate_'+str(i+1);ramp(candidate['scale']);b=measure('B'+str(i+1))
   stage='ramp_baseline';ramp(reference_scale);a=measure('A'+str(i+1)+'_return')
   reference={k:(previous[k]+a[k])/2 for k in ['pitch_rms_deg','pitch_rate_rms','speed_rms','yaw_rms_deg','current_rms_A']}
   ratio=score(b,reference);eligible=ratio<.95 and b['pitch_peak_deg']<max(previous['pitch_peak_deg'],a['pitch_peak_deg'])*1.25 and b['saturation_fraction']<.01
   trials.append(dict(candidate=i,score=ratio,eligible=eligible,metrics=b,reference=reference))
   event('comparison',candidate=i+1,score=ratio,eligible=eligible);save(dict(trials=trials));previous=a
  except ValueError as exc:
   event('rollback',reason=str(exc));ramp(reference_scale,False);pump(8,True)
   trials.append(dict(candidate=i,score=1e9,eligible=False,reason=str(exc)))
 eligible=[trial for trial in trials if trial['eligible']]
 chosen=min(eligible,key=lambda trial:trial['score']) if eligible else None
 if chosen:
  ramp(plan['candidates'][chosen['candidate']]['scale']);final=measure('selected_validation',60,10)
  validation_score=score(final,chosen['reference'])
  if validation_score>=.98:
   event('validation_rollback',score=validation_score);ramp(reference_scale);chosen=None
 else:
  validation_score=None;measure('reference_validation',60,10)
 stage='continuous_selected' if chosen else 'continuous_baseline';save(dict(trials=trials,chosen=chosen,validation_score=validation_score))
 event('TUNING_COMPLETE',selected_scale=current,validation_score=validation_score,folder=str(folder))
 commands=queue.Queue()
 def inputs():
  for line in sys.stdin:commands.put(line.strip())
 threading.Thread(target=inputs,daemon=True).start()
 while True:
  pump(.1,False)
  while not commands.empty():
   text=commands.get()
   if text=='QUIT':raise SystemExit
   if text:event('reply',command=text,reply=command(text))
  if last and last['fault']:
   event('fault',reply=command('FAULT'));stage='faulted';time.sleep(.5)
except Exception as exc:
 if isinstance(exc,ValueError) and last and last['fault']==0 and last['mode']==2:
  try:ramp(reference_scale,False)
  except Exception:pass
 event('ERROR',error=str(exc));save(dict(error=str(exc)))
 # No blind restart or fault clear. Preserve UART connection for diagnosis.
 while True:pump(.2,False)
