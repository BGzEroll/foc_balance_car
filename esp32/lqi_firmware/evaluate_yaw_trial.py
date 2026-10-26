"""Evaluate heading tracking and braking from one recorded yaw trial."""
import bisect,csv,json,math,pathlib,statistics,sys
p=pathlib.Path(sys.argv[1]);report=json.loads((p/'report.json').read_text())
def read(name):
 with (p/name).open() as f:return [{k:float(v) for k,v in r.items()} for r in csv.DictReader(f)]
s=read('samples.csv');y=read('yaw.csv');y=[r for r in y if r['time_us']>=report['start_us'] and r['phase']<4]
assert y,'Missing active yaw telemetry'
ts=[r['sample_us'] for r in s]
def interp(t,k):
 i=max(0,min(len(s)-2,bisect.bisect_right(ts,t)-1));u=(t-ts[i])/(ts[i+1]-ts[i]);return s[i][k]+u*(s[i+1][k]-s[i][k])
error=[];rates=[];hold_errors=[];overshoots=[];settles=[]
for r in y:
 error.append((interp(r['time_us'],'yaw')-r['target'])*180/math.pi)
 rates.append((interp(r['time_us'],'yaw_rate')-r['target_rate'])*180/math.pi)
for phase in range(4):
 leg=[r for r in y if r['phase']==phase];peak_rate=max(abs(r['target_rate']) for r in leg)
 moving=[r for r in leg if abs(r['target_rate'])>max(1e-4,peak_rate*.001)]
 if not moving:continue
 end_move=moving[-1]['time_us'];hold=[r for r in leg if r['time_us']>end_move+20000]
 direction=1 if phase in (0,3) else -1
 e=[(interp(r['time_us'],'yaw')-r['target'])*180/math.pi for r in hold]
 hold_errors.extend(e);overshoots.append(max([0]+[direction*v for v in e]))
 settled=None
 for i,r in enumerate(hold):
  window=[v for v in hold[i:] if v['time_us']<r['time_us']+500000]
  if window and window[-1]['time_us']-r['time_us']>=450000 and all(abs(interp(v['time_us'],'yaw')-v['target'])<math.radians(2) and abs(interp(v['time_us'],'yaw_rate'))<math.radians(3) for v in window):
   settled=(r['time_us']-end_move)*1e-6;break
 settles.append(settled)
rms=lambda v:math.sqrt(statistics.fmean(x*x for x in v))
first=s[0];enc0=(first['angle_a']+first['angle_b'])*.0325/.1325364
result=dict(label=p.name,amplitude_deg=report['amplitude'],move_seconds=report['move_seconds'],yaw_gains={k:s[-1][k] for k in ['yaw_kp','yaw_ki','yaw_kd']},tracking_rms_deg=rms(error),tracking_peak_deg=max(map(abs,error)),rate_tracking_rms_deg_s=rms(rates),hold_rms_deg=rms(hold_errors),max_braking_overshoot_deg=max(overshoots),settling_s_by_leg=settles,peak_relative_yaw_deg=max(abs(r['yaw']-first['yaw'])*180/math.pi for r in s),peak_encoder_yaw_deg=max(abs((r['angle_a']+r['angle_b'])*.0325/.1325364-enc0)*180/math.pi for r in s),peak_planar_m=report['peak_planar_encoder_odometry_m'],peak_command_A=report['peak_current_command_A'],peak_pitch_deg=report['peak_pitch_deg'],faults=sorted(set(r['fault'] for r in s)),scope='Tracking against IMU yaw; encoder heading is a cross-check, not absolute ground truth.')
(p/'yaw_metrics.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
