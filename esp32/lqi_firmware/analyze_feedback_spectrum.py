"""Closed-loop feedback decomposition over the common 48..65s recovery window."""
import csv,json,sys
from pathlib import Path
import numpy as np
folder=Path(sys.argv[1]);h=Path(__file__).resolve().parent
d=json.loads((folder/'dynamic_result.json').read_text())
design=json.loads((h/'lqi_design.json').read_text())
k=np.array(design['Kaug'])[0]*np.array(d['scale'])
rows=list(csv.DictReader((folder/'samples.csv').open()))
t=np.array([float(r['sample_us'])*1e-6-d['started_us']*1e-6 for r in rows])
fields=['distance','speed','pitch','pitch_rate','yaw','yaw_rate','velocity_integral','yaw_integral']
v=np.array([[float(r[n]) for n in fields] for r in rows])[(t>=48)&(t<65)]
v[:,0]-=d['origin_m'];v[:,2]-=design['hardware_pitch_trim_rad']
u=-v*k;n=len(u);f=np.fft.rfftfreq(n,.005)
y=np.fft.rfft(u-u.mean(0),axis=0);bands=[]
for lo,hi in [(.3,2),(2,8),(8,30),(30,80)]:
 z=y.copy();z[(f<lo)|(f>=hi)]=0;z=np.fft.irfft(z,n=n,axis=0)
 bands.append(dict(range_Hz=[lo,hi],component_rms_A=dict(zip(fields,np.sqrt(np.mean(z*z,0)).tolist())),common_rms_A=float(np.sqrt(np.mean(z.sum(1)**2)))))
p=abs(np.fft.rfft(v[:,3]-v[:,3].mean()))**2
ids=np.flatnonzero((f>.3)&(f<50));peak=float(f[ids[np.argmax(p[ids])]])
report=dict(peak_pitch_rate_frequency_Hz=peak,bands=bands,window_s=[48,65],scope='Correlated closed-loop feedback terms; RMS contributions do not add and this is not a causal plant identification.')
(folder/'feedback_spectrum.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
