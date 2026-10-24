"""Low-frequency settling trial using the already-open logger; no port reset."""
import argparse,csv,json,math,os,statistics,sys,time
from pathlib import Path
H=Path(__file__).resolve().parent
parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('source');parser.add_argument('--resume');parser.add_argument('--notch',action='store_true');parser.add_argument('--notch-mix',type=float,default=1.0);parser.add_argument('--factors',nargs='+',type=float,default=[.9,.82]);parser.add_argument('--candidate-index',type=int);parser.add_argument('--reference-report');parser.add_argument('--ignore-current-growth',action='store_true');parser.add_argument('--confirm-report');args=parser.parse_args()
source=Path(args.source);resume=json.loads(Path(args.resume).read_text()) if args.resume else None
base=resume['current'] if resume else json.loads((H/'selected_lqi.json').read_text())['scale']
checks=json.loads((H/'results/settling/screen.json').read_text())['cases']
assert all(c['nonlinear_pass'] and c['max_radius']<1 for c in checks)
if args.notch:
 assert not resume
 assert .1<=args.notch_mix<=1 and abs(args.notch_mix*10-round(args.notch_mix*10))<1e-6
 design=json.loads((H/'results/jitter_refine/notch_design.json').read_text())
 assert design['mode']==0 and design['max_closed_loop_radius']<1 and all(c['pass'] for c in design['nonlinear_cases'])
folder=H/'results'/('settling_'+time.strftime('%Y%m%d_%H%M%S'));folder.mkdir(parents=True)
fp=source.open();fields=next(csv.reader([fp.readline()]));fp.seek(0,2)
stream=(folder/'samples.csv').open('w',newline='');writer=csv.writer(stream);writer.writerow(['trial_stage',*fields,'band_rate','band_common_current'])
log=(folder/'events.jsonl').open('w',buffering=1);buf='';last=None;stage='attach';current=base.copy();history=[];rows=[];last_progress=0
notch_mix=0.0
scale_fields=['velocity_kp','velocity_ki','pitch_kd','pitch_kp','pitch_ki','yaw_kp','yaw_ki','yaw_kd']

class Band:
 def __init__(self):
  w=2*math.pi*.8/200;a=math.sin(w)/(2*.7);den=1+a
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
    if abs(r['distance'])>.15:raise ValueError('15cm displacement rollback margin')
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
 (folder/'report.json').write_text(json.dumps(dict(base=base,current=current,ignore_current_growth=args.ignore_current_growth,notch_mix=notch_mix,notch_design=design if args.notch else None,history=history,extra=extra,source=str(source),screen=checks),indent=2))

def measure(label,seconds=60):
 global stage,rows
 stage=label+'_settle';read(8);stage=label;rows=[];event('measuring',seconds=seconds);read(seconds)
 assert len(rows)>seconds*180,'Insufficient telemetry'
 q=[r['band_rate'] for r in rows];u=[r['band_common_current'] for r in rows]
 burst=[rms(q[i:i+200])>.15 for i in range(0,len(q)-199,200)]
 pitch=[(r['pitch']-r['target_pitch'])*180/math.pi for r in rows]
 result=dict(label=label,samples=len(rows),scale=current.copy(),notch_mix=notch_mix,revision=int(last['revision']),
  band_rate_rms=rms(q),band_common_current_rms=rms(u),burst_fraction=statistics.fmean(burst),
  rate_rms=rms([r['pitch_rate'] for r in rows]),pitch_rms_deg=rms(pitch),pitch_mean_deg=statistics.fmean(pitch),pitch_ac_std_deg=statistics.pstdev(pitch),pitch_peak_deg=max(map(abs,pitch)),
  current_peak_A=max(abs(r[k]) for r in rows for k in ('command_a','command_b')),distance_peak_m=max(abs(r['distance']) for r in rows),speed_rms=rms([r['speed'] for r in rows]),current_rms=rms([r[k] for r in rows for k in ('command_a','command_b')]),
  saturation_fraction=statistics.fmean(r['saturated'] for r in rows),faults=max(r['fault'] for r in rows))
 history.append(result);event('metrics',metrics=result);save();return result

def compare(b,a0,a1):
 ref={k:(a0[k]+a1[k])/2 for k in ['band_rate_rms','band_common_current_rms','current_rms','speed_rms','pitch_rms_deg','pitch_ac_std_deg','rate_rms','burst_fraction']}
 score=.5*(b['band_rate_rms']/max(ref['band_rate_rms'],.015))**2+.3*(b['speed_rms']/max(ref['speed_rms'],.003))**2+.2*(b['pitch_ac_std_deg']/max(ref['pitch_ac_std_deg'],.2))**2
 good=score<.8 and b['rate_rms']<max(.12,ref['rate_rms']*1.25) and (args.ignore_current_growth or b['current_rms']<max(.15,ref['current_rms']*1.2)) and b['pitch_peak_deg']<10 and b['saturation_fraction']<.005
 return dict(reference=ref,score=score,eligible=good,candidate=b)

try:
 read(.5);assert not args.notch and not resume
 assert max(abs(last[k]-v) for k,v in zip(scale_fields,base))<2e-6
 assert abs(last['position_kp'])<1e-6
 event('attached',folder=str(folder));trials=[];chosen=None;confirm=None
 if args.confirm_report:
  prior=json.loads(Path(args.confirm_report).read_text());trial=prior['extra']['trials'][-1]
  assert max(abs(v-w) for v,w in zip(prior['base'],base))<2e-6
  target=trial['candidate']['scale'];assert any(max(abs(v-w) for v,w in zip(target,c['scale']))<2e-6 for c in checks)
  event('confirmation_reference',prior_report=args.confirm_report,reference=trial['reference'],ignore_current_growth=args.ignore_current_growth)
  ramp(target);confirm=measure('B_validation',120);validation=compare(confirm,trial['reference'],trial['reference'])
  chosen=validation if validation['eligible'] else None
  if not chosen:ramp(base)
  save(dict(chosen=chosen,confirmation=confirm,prior_report=args.confirm_report,reference=trial['reference']));event('SETTLING_COMPLETE',chosen=chosen is not None,scale=current,folder=str(folder));sys.exit(0)
 if args.reference_report:
  p=Path(args.reference_report);assert time.time()-p.stat().st_mtime<120
  old=json.loads(p.read_text());a0=old['history'][-1];assert a0['label'].startswith('A') and a0['revision']==int(last['revision'])
  a0=a0.copy();a0['label']='A0_reused';history.append(a0);event('reference_reused',source=str(p),metrics=a0)
 else:a0=measure('A0',75)
 for index,c in enumerate(([checks[args.candidate_index]] if args.candidate_index else checks[1:])):
  stage='ramp_candidate';ramp(c['scale']);b=measure('B'+str(index+1),75)
  stage='ramp_reference';ramp(base);a1=measure('A'+str(index+1)+'_return',75)
  trial=compare(b,a0,a1);trials.append(trial);event('comparison',result=trial);save(dict(trials=trials));a0=a1
 eligible=[t for t in trials if t['eligible']]
 if eligible:
  chosen=min(eligible,key=lambda t:t['score']);ramp(chosen['candidate']['scale']);confirm=measure('B_validation',120)
  validation=compare(confirm,chosen['reference'],chosen['reference'])
  event('validation',result=validation)
  if not validation['eligible']:ramp(base);chosen=None
 save(dict(trials=trials,chosen=chosen,confirmation=confirm));event('SETTLING_COMPLETE',chosen=chosen is not None,scale=current,folder=str(folder))
except Exception as exc:
 event('ERROR',reason=str(exc))
 if last and not last['fault'] and last['mode']==2:
  try:ramp(base,False)
  except Exception as rollback:event('ROLLBACK_ERROR',reason=str(rollback))
 save(dict(error=str(exc)));raise
finally:stream.flush();stream.close();fp.close();log.close()
