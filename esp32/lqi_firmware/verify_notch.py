"""Exercise the actual firmware filter, including startup, DC and rejection."""
import json,subprocess
from pathlib import Path
import numpy as np
H=Path(__file__).resolve().parent
d=json.loads((H/'results/jitter_refine/notch_design.json').read_text())
exe=H/'results/verify_notch.exe'
subprocess.run(['C:/software/mingw64/bin/g++.exe','-std=c++17','-O2','-Wall','-Wextra','-Werror','-static','-I',str(H.parent/'balance_car/user_lib'),str(H/'verify_notch.cpp'),'-o',str(exe)],check=True)
def run(x):
 p=subprocess.run([str(exe)],input='\n'.join(format(v,'.9g') for v in x),text=True,capture_output=True,check=True)
 return np.fromstring(p.stdout,sep='\n')
rng=np.random.default_rng(24);x=rng.uniform(-12,12,20000).astype(np.float32);y=run(x)
b=np.array(d['b'],dtype=np.float32).astype(float);a=np.array(d['a'],dtype=np.float32).astype(float)
x1=x2=y1=y2=float(x[0]);ref=[y1]
for u in x[1:]:
 v=b[0]*u+b[1]*x1+b[2]*x2-a[1]*y1-a[2]*y2
 ref.append(v);x2=x1;x1=u;y2=y1;y1=v
err=float(max(abs(y-np.array(ref))));assert err<.0002,err
dc=run(np.full(2000,3.0));assert max(abs(dc-3))<1e-5
t=np.arange(4000)/200
ratios={}
for f in [1,3,12.5]:
 u=np.sin(2*np.pi*f*t);v=run(u);ratios[f]=float(np.std(v[2000:])/np.std(u[2000:]))
assert ratios[12.5]<.0001 and ratios[3]>.99 and ratios[1]>.998
report=dict(samples=len(x),maximum_reference_error=err,constant_start_error=float(max(abs(dc-3))),amplitude_ratios=ratios,result='PASS',scope='Actual C++ header; no powered evidence')
(H/'results/jitter_refine/native_notch.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
