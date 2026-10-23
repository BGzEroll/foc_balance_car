import json,subprocess,os
from pathlib import Path
import numpy as np
H=Path(__file__).resolve().parent;R=H.parents[1];P=R/'esp32/balance_car/user_lib'
os.environ['PATH']=r'C:\software\mingw64\bin;'+os.environ['PATH']
d=json.loads((H/'lqi_design.json').read_text());K=np.array(d['Kaug']);rng=np.random.default_rng(24);n=16000
v=np.zeros((n,23));v[:,0]=rng.uniform(-.5,.5,n);v[:,1]=rng.uniform(-.35,.35,n)+d['hardware_pitch_trim_rad'];v[:,2]=rng.uniform(-2,2,n);v[:,3]=rng.uniform(-2.8,2.8,n);v[:,4]=rng.uniform(-2,2,n)
v[:,5:9]=rng.uniform(-.2,.2,(n,4));v[:,9]=rng.uniform(-.2,.2,n);v[:,10]=rng.uniform(-.5,.5,n);v[:,11]=rng.uniform(-.3,.3,n);v[:,12]=rng.uniform(-1,1,n);v[:,13]=rng.integers(0,2,n);v[:,14]=rng.choice([.7,2,6,8],n);v[:,15:]=rng.uniform(.5,1.5,(n,8));v[:1000,15:]=1
v=v.astype(np.float32).astype(float)
exe=H/'results/verify_lqi.exe';subprocess.run([r'C:\software\mingw64\bin\g++.exe','-std=c++17','-O2','-Wall','-Wextra','-Werror','-I',str(P),str(H/'native_test.cpp'),str(P/'balance.cpp'),'-o',str(exe)],check=True)
run=subprocess.run([str(exe)],input='\n'.join(' '.join(format(a,'.9g') for a in row) for row in v),capture_output=True,text=True,check=True)
got=np.array([[float(a) for a in line.split()] for line in run.stdout.splitlines()]);expected=[]
for row in v:
 speed,pitch,q,yaw,w,refpos,refspeed,refyaw,refw,dist,prev,xi,yi,first,cap=row[:15]
 if not first:dist+=.005*(prev+speed)*.5
 ey=np.arctan2(np.sin(yaw-refyaw),np.cos(yaw-refyaw));e=[dist-refpos,speed-refspeed,pitch-d['hardware_pitch_trim_rad'],q,ey,w-refw,xi,yi]
 effort=-(K*row[15:])@e;raw=np.array([(effort[0]-effort[1])/2,(effort[0]+effort[1])/2]);cap=min(6,abs(cap));sat=np.max(abs(raw))>cap
 if not sat:xi=np.clip(xi+.005*e[0],-.3,.3);yi=np.clip(yi+.005*ey,-1,1)
 expected.append([*np.clip(raw,-cap,cap),dist,speed,xi,yi,0,float(sat)])
err=np.max(abs(got-np.array(expected)),axis=0);assert got.shape==(n,8);assert max(err[:7])<2e-5;assert err[7]==0
assert np.max(abs(got[:,:2]))<=6+1e-6
report=dict(samples=n,max_errors=err.tolist(),saturated_samples=int(got[:,-1].sum()),current_cap_A=6,scope='Compiled firmware versus MATLAB matrix reference with random online column scales, first sample, distance integration, yaw wrap, 6A caps and integral freezing. No powered validation.')
(H/'results/online_lqi/native_equivalence.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
