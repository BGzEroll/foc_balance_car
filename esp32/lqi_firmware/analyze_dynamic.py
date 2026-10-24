"""Offline metrics for the compiled bounded trajectory; no hardware access."""
import csv,json,sys
from pathlib import Path
import numpy as np
H=Path(__file__).resolve().parent;folder=Path(sys.argv[1]) if len(sys.argv)>1 else H/'results/dynamic/first_run'
d=json.loads((folder/'dynamic_result.json').read_text());rows=list(csv.DictReader((folder/'samples.csv').open()))
t=np.array([float(r['sample_us']) for r in rows])*1e-6-d['started_us']*1e-6
x=np.array([float(r['distance']) for r in rows])-d['origin_m'];v=np.array([float(r['speed']) for r in rows]);pitch=np.array([float(r['pitch'])-float(r['target_pitch']) for r in rows])*180/np.pi
ref=np.zeros(len(rows));rv=np.zeros(len(rows))
for k,(a,b) in enumerate(zip([0,.05,0,-.05],[.05,0,-.05,0])):
 m=(t>=12*k)&(t<12*(k+1));u=np.clip((t[m]-12*k)/6,0,1);h=u**3*(10-15*u+6*u*u);dh=30*u*u*(1-u)**2/6
 ref[m]=a+(b-a)*h;rv[m]=(b-a)*dh
trim=json.loads((H/'lqi_design.json').read_text())['hardware_pitch_trim_rad'];body_pitch=(np.array([float(r['pitch']) for r in rows])-trim)*180/np.pi;reference_pitch=(np.array([float(r['target_pitch']) for r in rows])-trim)*180/np.pi
active=(t>=0)&(t<=48);recovery=(t>=48)&(t<=65)
rms=lambda a:float(np.sqrt(np.mean(a*a)))
segments=[]
for k in range(4):
 m=(t>=12*k)&(t<12*(k+1));hold=(t>=12*k+10)&(t<12*(k+1))
 segments.append(dict(phase=k,position_error_rms_m=rms(x[m]-ref[m]),position_min_m=float(x[m].min()),position_max_m=float(x[m].max()),last_2s_position_mean_m=float(x[hold].mean()),last_2s_speed_rms_m_s=rms(v[hold])))
q=recovery&(abs(x)<.005)&(abs(v)<.01);stable=None
for i in np.flatnonzero(recovery):
 end=np.searchsorted(t,t[i]+2)
 if end<len(t) and t[end]<=65 and np.all(q[i:end+1]):stable=float(t[i]-48);break
report=dict(position_tracking_rms_m=rms(x[active]-ref[active]),position_tracking_peak_m=float(max(abs(x[active]-ref[active]))),
 speed_tracking_rms_m_s=rms(v[active]-rv[active]),actual_peak_speed_m_s=float(max(abs(v[active]))),reference_peak_speed_m_s=float(max(abs(rv))),
 peak_pitch_error_deg=float(max(abs(pitch[active]))),peak_body_pitch_deg=float(max(abs(body_pitch[active]))),peak_encoder_displacement_m=float(max(abs(x))),
 return_settle_to_5mm_and_1cms_for_2s_s=stable,return_recovery_speed_rms_m_s=rms(v[recovery]),return_recovery_position_rms_m=rms(x[recovery]),
 target_speed_telemetry_max_error=float(max(abs(np.array([float(r['target_speed']) for r in rows])[active]-rv[active]))),segments=segments,
 scope='One slow position-reference trajectory. Encoder displacement only, not independent ground motion or abrupt speed-step validation.')
(folder/'analysis.json').write_text(json.dumps(report,indent=2));np.savetxt(folder/'tracking.csv',np.column_stack([t,x,ref,v,rv,pitch,body_pitch,reference_pitch]),delimiter=',',header='time_s,position_m,reference_m,speed_m_s,reference_speed_m_s,pitch_error_deg,body_pitch_deg,reference_pitch_deg',comments='')
print(json.dumps(report,indent=2))
