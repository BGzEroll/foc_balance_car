"""Low-frequency settling trial using the already-open logger; no port reset."""
import argparse,csv,json,math,os,statistics,sys,time
from pathlib import Path
H=Path(__file__).resolve().parent
parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('source');parser.add_argument('--resume');parser.add_argument('--notch',action='store_true');parser.add_argument('--notch-mix',type=float,default=1.0);parser.add_argument('--factors',nargs='+',type=float,default=[.9,.82]);parser.add_argument('--candidate-index',type=int);parser.add_argument('--reference-report');parser.add_argument('--ignore-current-growth',action='store_true');parser.add_argument('--confirm-report');parser.add_argument('--friction-force',type=float);parser.add_argument('--integral-limit',type=float);parser.add_argument('--encoder-ab-us',type=int);parser.add_argument('--excursion-m',type=float,default=.05);parser.add_argument('--minimum-m',type=float);args=parser.parse_args()
assert .05<=args.excursion_m<=.14
assert args.minimum_m is None or 0<args.minimum_m<=args.excursion_m
source=Path(args.source);resume=json.loads(Path(args.resume).read_text()) if args.resume else None
base=resume['current'] if resume else json.loads((H/'selected_lqi.json').read_text())['scale']
checks=json.loads((H/'results/dynamic/tuning_screen.json').read_text())['cases'] if args.candidate_index else []
assert all(c['nonlinear_pass'] and c['max_radius']<1 for c in checks)
if args.notch:
 assert not resume
 assert .1<=args.notch_mix<=1 and abs(args.notch_mix*10-round(args.notch_mix*10))<1e-6
 design=json.loads((H/'results/jitter_refine/notch_design.json').read_text())
 assert design['mode']==0 and design['max_closed_loop_radius']<1 and all(c['pass'] for c in design['nonlinear_cases'])
folder=H/'results'/('dynamic_'+time.strftime('%Y%m%d_%H%M%S'));folder.mkdir(parents=True)
fp=source.open();fields=next(csv.reader([fp.readline()]));fp.seek(0,2)
stream=(folder/'samples.csv').open('w',newline='');writer=csv.writer(stream);writer.writerow(['trial_stage',*fields,'band_rate','band_common_current'])
log=(folder/'events.jsonl').open('w',buffering=1);buf='';last=None;stage='attach';current=base.copy();history=[];rows=[];last_progress=0
guard_origin=None
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
 global buf,last,last_progress,guard_origin
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
   if guard_origin is None:guard_origin=r['distance']
   r['band_rate']=qfilter.update(r['pitch_rate']);r['band_common_current']=ifilter.update(r['command_a']+r['command_b'])
   writer.writerow([stage,*values,r['band_rate'],r['band_common_current']]);rows.append(r)
   if guard:
    if r['fault'] or r['mode']!=2 or int(r['flags'])&6!=6:raise RuntimeError('Controller stopped: fault='+str(r['fault']))
    if abs(r['distance']-guard_origin)>.19:raise ValueError('Unexpected 19cm displacement from test attachment')
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

import re
command_log=(source.parent/'commands.log').open();command_log.seek(0,2)
def command(text):
 command_log.seek(0,2)
 fd=os.open('/dev/ttyACM0',os.O_RDWR|os.O_NOCTTY|os.O_NONBLOCK)
 try:os.write(fd,(text+'\n').encode())
 finally:os.close(fd)
 deadline=time.monotonic()+3;pending=''
 while time.monotonic()<deadline:
  read(.05);pending+=command_log.read()
  for line in pending.splitlines():
   if line.startswith(text.split()[0]+' '):
    event('reply',command=text,reply=line)
    return dict(re.findall(r'(\w+)=([^ ]+)',line))
 raise TimeoutError(text)
try:
 read(.5);assert max(abs(last[k]-v) for k,v in zip(scale_fields,base))<2e-6
 event('attached',folder=str(folder));stage='baseline';read(10)
 if args.encoder_ab_us is not None:
  assert args.encoder_ab_us==0 or 1000<=args.encoder_ab_us<=10000
  assert command('ENCAB '+str(args.encoder_ab_us))['selected_us']==str(args.encoder_ab_us)
  stage='observer_settle';read(5)
 if args.friction_force is not None:
  assert 0<=args.friction_force<=.35
  applied=float(command('LQI').get('friction_ff_N',0))
  while abs(applied-args.friction_force)>1e-6:
   step=round(applied+max(-.05,min(.05,args.friction_force-applied)),6)
   reply=command('LQIFF '+format(step,'.6f'));assert reply['accepted']=='1'
   applied=float(reply['force_N'])
  assert abs(float(command('LQI')['friction_ff_N'])-args.friction_force)<1e-6

 if args.integral_limit is not None:
  assert 0<=args.integral_limit<=.12
  reply=command('DYNINT '+format(args.integral_limit,'.6f'));assert reply['accepted']=='1'
  assert abs(float(command('DYNSTAT')['integral_limit'])-args.integral_limit)<1e-6
 if args.candidate_index:
  target=checks[args.candidate_index]['scale'];ramp(target);stage='candidate_settle';read(8)
 stage='waiting_stationary';deadline=time.monotonic()+90;rows=[]
 while time.monotonic()<deadline:
  read(.1)
  if len(rows)>=200:
   window=rows[-200:]
   if max(abs(r['speed']) for r in window)<.015 and max(abs(r['distance']) for r in window)<.015 and max(r['pitch'] for r in window)-min(r['pitch'] for r in window)<.0174533:break
 else:raise RuntimeError('Candidate did not reach stationary start gate within 90s')
 event('stationary_gate_passed',distance=last['distance'],speed=last['speed'])
 assert command('DYNSTAT')['armed']=='0'
 assert command('DYNTEST '+format(args.excursion_m,'.6f'))['accepted']=='1';st=command('DYNSTAT');assert st['armed']=='1'
 began=int(st['began_us']);origin=float(st['origin']);stage='motion_and_recovery';rows=[];read(65)
 st=command('DYNSTAT');assert st['aborted']=='0' and st['hard_stop']=='0' and st['phase']=='4',st
 peak=max(abs(r['distance']-origin) for r in rows);assert peak<.17
 assert command('DYNEND')['accepted']=='1';assert command('DYNSTAT')['armed']=='0'
 stage='post_test_balance';read(10)
 forward=max(r['distance']-origin for r in rows);backward=max(origin-r['distance'] for r in rows)
 result=dict(excursion_m=args.excursion_m,minimum_m=args.minimum_m,minimum_achieved=args.minimum_m is None or min(forward,backward)>=args.minimum_m,forward_excursion_m=forward,backward_excursion_m=backward,started_us=began,origin_m=origin,peak_encoder_displacement_m=peak,status=st,scale=current.copy(),friction_force_N=args.friction_force,motion_integral_limit=args.integral_limit,encoder_ab_us=args.encoder_ab_us,
  current_peak_A=max(abs(r[k]) for r in rows for k in ('command_a','command_b')),
  pitch_peak_deg=max(abs(r['pitch']-r['target_pitch'])*180/math.pi for r in rows),body_pitch_peak_deg=max(abs(r['pitch'])*180/math.pi for r in rows),
  saturated_fraction=statistics.fmean(r['saturated'] for r in rows),scope='One forward/return/backward/return trajectory with recorded amplitude; encoder-based position, no independent ground measurement.')
 (folder/'dynamic_result.json').write_text(json.dumps(result,indent=2))
 if args.candidate_index:stage='ramp_reference';ramp(base)
 if args.friction_force is not None:
  applied=args.friction_force
  while applied>1e-6:
   applied=round(max(0,applied-.05),6);assert command('LQIFF '+format(applied,'.6f'))['accepted']=='1'
 if args.integral_limit is not None:assert command('DYNINT 0')['accepted']=='1'
 if args.encoder_ab_us is not None:command('ENCAB 0')
 event('DYNAMIC_TEST_COMPLETE',result=result,folder=str(folder))
except Exception as exc:
 event('ERROR',reason=str(exc));(folder/'dynamic_result.json').write_text(json.dumps(dict(error=str(exc),last=last),indent=2))
 try:
  st=command('DYNSTAT')
  if st.get('armed')=='1':
   if st.get('phase') not in ('4','5'):command('DYNABORT');read(.1,False)
   command('DYNEND')
 except Exception:pass
 if args.encoder_ab_us is not None:
  try:command('ENCAB 0')
  except Exception:pass
 if args.integral_limit is not None:
  try:command('DYNINT 0')
  except Exception:pass
 if args.friction_force is not None:
  try:
   applied=float(command('LQI').get('friction_ff_N',0))
   while applied>1e-6:
    applied=round(max(0,applied-.05),6);command('LQIFF '+format(applied,'.6f'))
  except Exception:pass
 if last and not last['fault'] and last['mode']==2:
  try:
   current=[last[k] for k in scale_fields];ramp(base,False)
  except Exception:pass
 raise
finally:stream.flush();stream.close();fp.close();log.close();command_log.close()
