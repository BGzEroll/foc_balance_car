"""Offline spectral decomposition of measured LQI feedback; no hardware access."""
import csv,json,sys
from pathlib import Path
import numpy as np
H=Path(__file__).resolve().parent
source=Path(sys.argv[1]) if len(sys.argv)>1 else H/'results/jitter/before.csv'
out=H/'results/jitter/spectral_diagnosis.json'
allrows=[r for r in csv.DictReader(source.open()) if r['mode']=='2' and r['fault']=='0']
revision=allrows[-1]['revision'];rows=[r for r in allrows if r['revision']==revision][-60000:]
assert all(abs(float(r.get('position_kp',0)))<1e-6 for r in rows),'Feedback decomposition requires notch disabled'
names=['distance','speed','pitch','pitch_rate','yaw','yaw_rate','velocity_integral','yaw_integral']
slots=['velocity_kp','velocity_ki','pitch_kd','pitch_kp','pitch_ki','yaw_kp','yaw_ki','yaw_kd']
x=np.array([[float(r[k]) for k in names] for r in rows]);x[:,2]-=float(rows[0]['target_pitch'])
scale=np.array([[float(r[k]) for k in slots] for r in rows]);K=np.array(json.loads((H/'lqi_design.json').read_text())['Kaug'])[0]
parts=-x*scale*K
u=np.array([[float(r['command_a'])+float(r['command_b']),-float(r['iq_a'])+float(r['iq_b'])] for r in rows])
n=800;f=np.fft.rfftfreq(n,.005);cross=np.zeros((n//2+1,10),complex);power=np.zeros((n//2+1,10));windows=0
for i in range(0,len(x)-n,n//2):
 if float(rows[i+n-1]['sample_us'])-float(rows[i]['sample_us'])>n*5050:continue
 a=np.column_stack([parts[i:i+n],u[i:i+n]]);a-=a.mean(axis=0)
 F=np.fft.rfft(a*np.hanning(n)[:,None],axis=0);cross+=F*np.conj(F[:,8,None]);power+=abs(F)**2;windows+=1
band=(f>=9)&(f<=15);peak=np.where(band)[0][np.argmax(power[band,8])]
report=dict(source=str(source),revision=int(revision),samples=len(rows),welch_windows=windows,peak_Hz=float(f[peak]),
 terms={name:dict(command_inphase_projection=float(cross[peak,j].real/power[peak,8]),phase_to_command_deg=float(np.angle(cross[peak,j],deg=True))) for j,name in enumerate(names)},
 actual_current_to_command=dict(magnitude=float(abs(cross[peak,9]/power[peak,8])),phase_deg=float(np.angle(cross[peak,9],deg=True)),coherence=float(abs(cross[peak,9])**2/(power[peak,8]*power[peak,9]))),
 limitation='Closed-loop spectral decomposition, not independent causal fractions or full plant identification. 200Hz logging does not resolve high-frequency ADC/current-loop behavior.')
out.write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
