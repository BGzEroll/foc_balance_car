"""Run a small matched bench matrix, stopping progression on any failed gate."""
from pathlib import Path
import argparse,csv,json,statistics,subprocess,sys
r=Path(__file__).resolve().parent;p=argparse.ArgumentParser();p.add_argument('mode',choices=['poll','event','merged']);a=p.parse_args()
steps=[('speed10',10,.03,.3),('speed27p5',27.5,.04,.4)]
if a.mode!='poll':steps+=[('speed_minus27p5',-27.5,.04,.4)]
results=[]
for label,w,ratio,cap in steps:
 name=a.mode+'_'+label;print('BEGIN '+name,flush=True)
 cmd=[sys.executable,str(r/'run.py'),name,'--mask','3','--zero','1','-1','5.330672807','--zero','2','1','2.164932114','--limit',str(ratio),str(cap),'--command',f'SPEED {w} 4000 25']
 with (r/(name+'_runner.log')).open('w') as out:ret=subprocess.run(cmd,stdout=out,stderr=subprocess.STDOUT)
 if ret.returncode:raise SystemExit('STOPPED: '+name)
 j=json.loads((r/(name+'.json')).read_text());m=j['meta'];rows=list(csv.DictReader((r/(name+'.csv')).open()))
 result={'name':name,'target_rad_s':w,'meta':m,'sched':j['sched'],'metric':j['metric'],'latency':j['latency'],'speeds':[]}
 ok=m['reason']==1 and m['cmp_errors']==0 and m['fault_seen']==0
 for motor in [1,2]:
  q=[x for x in rows if int(x['motor'])==motor and int(x['stage'])==2 and (int(x['us'])-m['profile_start'])*1e-6>1]
  t=[(int(x['us'])-m['profile_start'])*1e-6 for x in q];v=[float(x['angle'])*j['setup']['direction'][motor-1] for x in q]
  tm=statistics.mean(t);vm=statistics.mean(v);speed=sum((u-tm)*(z-vm) for u,z in zip(t,v))/sum((u-tm)**2 for u in t)
  result['speeds'].append(speed)
  ok=ok and abs(speed-w)<max(.5,.1*abs(w)) and m['full_rate_valid'][motor-1]/max(1,m['full_rate_active'][motor-1])>=.999 and m['max_sample_gap_us'][motor-1]<=800 and m['max_encoder_age_us'][motor-1]<1500
  metric=next(x for x in j['metric'] if x['motor']==motor);ok=ok and metric['iq_error_rms_A']<.15
 result['progression_pass']=bool(ok);results.append(result)
 (r/(a.mode+'_summary.json')).write_text(json.dumps(results,indent=2));print(json.dumps(result),flush=True)
 if not ok:raise SystemExit('STOPPED: failed progression gate: '+name)
print('MATRIX_COMPLETE',a.mode,flush=True)
