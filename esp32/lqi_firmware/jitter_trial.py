"""Single-column live micro-jitter trial using the already-open logger; no port reset."""
import argparse,csv,json,math,os,statistics,sys,time
from pathlib import Path
H=Path(__file__).resolve().parent
parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('source');parser.add_argument('--resume');parser.add_argument('--notch',action='store_true');parser.add_argument('--notch-mix',type=float,default=1.0);parser.add_argument('--factors',nargs='+',type=float,default=[.9,.82]);args=parser.parse_args()
source=Path(args.source);resume=json.loads(Path(args.resume).read_text()) if args.resume else None
base=resume['current'] if resume else json.loads((H/'selected_lqi.json').read_text())['scale']
checks=json.loads((H/'results/jitter/screen.json').read_text())['cases']
assert all(c['nonlinear_pass'] and c['max_radius']<1 for c in checks)
if args.notch:
 assert not resume
 assert .1<=args.notch_mix<=1 and abs(args.notch_mix*10-round(args.notch_mix*10))<1e-6
 design=json.loads((H/'results/jitter_refine/notch_design.json').read_text())
 assert design['mode']==0 and design['max_closed_loop_radius']<1 and all(c['pass'] for c in design['nonlinear_cases'])
folder=H/'results'/('jitter_'+time.strftime('%Y%m%d_%H%M%S'));folder.mkdir(parents=True)
fp=source.open();fields=next(csv.reader([fp.readline()]));fp.seek(0,2)
stream=(folder/'samples.csv').open('w',newline='');writer=csv.writer(stream);writer.writerow(['trial_stage',*fields,'band_rate','band_common_current'])
log=(folder/'events.jsonl').open('w',buffering=1);buf='';last=None;stage='attach';current=base.copy();history=[];rows=[];last_progress=0
notch_mix=0.0
scale_fields=['velocity_kp','velocity_ki','pitch_kd','pitch_kp','pitch_ki','yaw_kp','yaw_ki','yaw_kd']

class Band:
 def __init__(self):
  w=2*math.pi*11.5/200;a=math.sin(w)/(2*.8);den=1+a
  self.b0=a/den;self.b2=-a/den;self.a1=-2*math.cos(w)/den;self.a2=(1-a)/den
  self.x1=self.x2=self.y1=self.y2=0
 def update(self,x):
  y=self.b0*x+self.b2*self.x2-self.a1*self.y1-self.a2*self.y2
  self.x2=self.x1;self.x1=x;self.y2=self.y1;self.y1=y;return y
qfilter=Band();ifilter=Band()

def event(kind,**kw):
 item=dict(time=time.time(),stage=stage,kind=kind,**kw);log.write(json.dumps(item)+'\n');print(json.dumps(item),flush=True)

def read(seconds,guard=True):
 global buf,last,last_progress
 end=time.monotonic()+seconds;seen=time.monotonic()
 while time.monotonic()<end:
  text=fp.read(65536)
  if not text:
   if time.monotonic()-seen>2:raise RuntimeError('Telemetry logger stalled')
   time.sleep(.02);continue
  seen=time.monotonic();buf+=text
  while '\n' in buf:
   line,buf=buf.split('\n',1);values=next(csv.reader([line]))
   if len(values)!=len(fields):raise RuntimeError('Incomplete CSV record')
   r={k:(v if k=='stage' else float(v)) for k,v in zip(fields,values)};last=r
   r['band_rate']=qfilter.update(r['pitch_rate']);r['band_common_current']=ifilter.update(r['command_a']+r['command_b'])
   writer.writerow([stage,*values,r['band_rate'],r['band_common_current']]);rows.append(r)
   if guard:
    if r['fault'] or r['mode']!=2 or int(r['flags'])&6!=6:raise RuntimeError('Controller stopped: fault='+str(r['fault']))
    if abs(r['pitch'])>.2618 or abs(r['speed'])>.35:raise ValueError('Trial tilt/speed guard')
  if time.monotonic()-last_progress>10:
   event('live',fault=int(last['fault']),pitch_deg=last['pitch']*180/math.pi,revision=int(last['revision']));last_progress=time.monotonic()
 stream.flush()

def ramp(target,guard=True):
 global current,notch_mix
 if args.notch:
  while abs(notch_mix-target)>1e-6:
   new=round(notch_mix+max(-.1,min(.1,target-notch_mix)),6)
   fd=os.open('/dev/ttyACM0',os.O_RDWR|os.O_NOCTTY|os.O_NONBLOCK)
   try:os.write(fd,('LQINOTCH '+str(new)+'\n').encode())
   finally:os.close(fd)
   read(.4,guard)
   if abs(last['position_kp']-new)>2e-6:raise RuntimeError('Notch mix not observed in telemetry')
   notch_mix=new;event('notch_mix',mix=new,revision=int(last['revision']))
  return
 target=[round(v,6) for v in target]
 while max(abs(a-b) for a,b in zip(current,target))>1e-6:
  new=[round(a+max(-.04,min(.04,b-a)),6) for a,b in zip(current,target)]
  command='LQISET '+' '.join(format(v,'.6f') for v in new);assert len(command)<96
  fd=os.open('/dev/ttyACM0',os.O_RDWR|os.O_NOCTTY|os.O_NONBLOCK)
  try:os.write(fd,(command+'\n').encode())
  finally:os.close(fd)
  read(.4,guard)
  if max(abs(last[k]-v) for k,v in zip(scale_fields,new))>2e-6:raise RuntimeError('Parameters not observed in telemetry')
  current=new;event('parameters',scale=current,revision=int(last['revision']))

def rms(x):return math.sqrt(statistics.fmean(v*v for v in x))
def save(extra=None):
 (folder/'report.json').write_text(json.dumps(dict(base=base,current=current,notch_mix=notch_mix,notch_design=design if args.notch else None,history=history,extra=extra,source=str(source),screen=checks),indent=2))

def measure(label,seconds=60):
 global stage,rows
 stage=label+'_settle';read(8);stage=label;rows=[];event('measuring',seconds=seconds);read(seconds)
 assert len(rows)>seconds*180,'Insufficient telemetry'
 q=[r['band_rate'] for r in rows];u=[r['band_common_current'] for r in rows]
 burst=[rms(q[i:i+200])>.15 for i in range(0,len(q)-199,200)]
 pitch=[(r['pitch']-r['target_pitch'])*180/math.pi for r in rows]
 result=dict(label=label,samples=len(rows),scale=current.copy(),notch_mix=notch_mix,revision=int(last['revision']),
  band_rate_rms=rms(q),band_common_current_rms=rms(u),burst_fraction=statistics.fmean(burst),
  pitch_rms_deg=rms(pitch),pitch_mean_deg=statistics.fmean(pitch),pitch_ac_std_deg=statistics.pstdev(pitch),pitch_peak_deg=max(map(abs,pitch)),
  speed_rms=rms([r['speed'] for r in rows]),current_rms=rms([r[k] for r in rows for k in ('command_a','command_b')]),
  saturation_fraction=statistics.fmean(r['saturated'] for r in rows),faults=max(r['fault'] for r in rows))
 history.append(result);event('metrics',metrics=result);save();return result

def compare(b,a0,a1):
 ref={k:(a0[k]+a1[k])/2 for k in ['band_rate_rms','band_common_current_rms','current_rms','speed_rms','pitch_rms_deg','burst_fraction']}
 ratio=b['band_rate_rms']/max(ref['band_rate_rms'],1e-5)
 good=ratio<.8 and b['current_rms']<ref['current_rms']*1.2 and b['speed_rms']<max(.03,ref['speed_rms']*1.3) and b['pitch_rms_deg']<max(3,ref['pitch_rms_deg']*1.35) and b['saturation_fraction']<.005
 return dict(reference=ref,rate_ratio=ratio,eligible=good,candidate=b)

try:
 read(.5);assert max(abs(last[k]-v) for k,v in zip(scale_fields,base))<2e-6
 if args.notch:assert abs(last['position_kp'])<1e-6
 event('attached',folder=str(folder))
 if resume:
  assert resume['extra']['chosen'];a0=resume['extra']['confirmation']
  assert max(abs(v-w) for v,w in zip(a0['scale'],base))<2e-6
  event('reuse_recent_baseline',metrics=a0)
 else:a0=measure('A0')
 trials=[];chosen=None
 for factor in ([1] if args.notch else args.factors):
  target=args.notch_mix if args.notch else base.copy()
  if not args.notch:
   target[3]*=factor
   assert any(max(abs(v-w) for v,w in zip(target,c['scale']))<2e-6 for c in checks),'Target was not screened in MATLAB'
  stage='ramp_candidate';ramp(target);b=measure('B_notch' if args.notch else 'B_rate_'+str(factor))
  stage='ramp_reference';ramp(0.0 if args.notch else base);a1=measure('A_return_'+str(factor));trial=compare(b,a0,a1);trials.append(trial)
  event('comparison',result=trial);save(dict(trials=trials))
  if trial['eligible']:chosen=trial;break
  a0=a1
 if chosen:
  ramp(args.notch_mix if args.notch else chosen['candidate']['scale']);confirm=measure('B_validation',90)
  ratio=confirm['band_rate_rms']/max(chosen['reference']['band_rate_rms'],1e-5)
  if ratio>.85 or confirm['current_rms']>chosen['reference']['current_rms']*1.25 or confirm['pitch_rms_deg']>max(3,chosen['reference']['pitch_rms_deg']*1.35):
   event('validation_failed',rate_ratio=ratio);ramp(0.0 if args.notch else base);chosen=None
 else:confirm=None
 save(dict(trials=trials,chosen=chosen,confirmation=confirm));event('JITTER_TRIAL_COMPLETE',chosen=chosen is not None,scale=current,folder=str(folder))
except Exception as exc:
 event('ERROR',reason=str(exc))
 if last and not last['fault'] and last['mode']==2:
  try:ramp(0.0 if args.notch else base,False)
  except Exception as rollback:event('ROLLBACK_ERROR',reason=str(rollback))
 save(dict(error=str(exc)));raise
finally:stream.flush();stream.close();fp.close();log.close()
