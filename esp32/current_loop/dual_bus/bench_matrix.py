"""Sequential, bounded dual bench tests. Any failed gate prevents escalation."""
from pathlib import Path
import argparse,csv,json,math,statistics,subprocess,sys
r=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--final',action='store_true');args=p.parse_args()
steps=[('dual_speed05',5,.025,.2),('dual_speed10',10,.03,.3),('dual_speed_minus10',-10,.03,.3),('dual_speed15',15,.04,.4),('dual_speed20',20,.04,.4),('dual_speed27p5',27.5,.04,.4),('dual_speed_minus27p5',-27.5,.04,.4)]
if args.final:steps=[(name+'_v18',w,ratio,cap) for name,w,ratio,cap in steps if w in [10,27.5,-27.5]]
summary_name='matrix_summary_v18.json' if args.final else 'matrix_summary.json'
summary=[]
for name,w,ratio,cap in steps:
 args=[sys.executable,str(r/'run_dual.py'),name,'--mask','3','--zero','1','-1','5.330672807','--zero','2','1','2.164932114','--limit',str(ratio),str(cap),'--command',f'SPEED {w} 4000 25']
 print('BEGIN '+name,flush=True)
 with (r/(name+'_runner.log')).open('w') as out:result=subprocess.run(args,stdout=out,stderr=subprocess.STDOUT)
 if result.returncode:raise SystemExit('STOPPED: runner failed: '+name)
 j=json.loads((r/(name+'.json')).read_text());m=j['meta'];data=list(csv.DictReader((r/(name+'.csv')).open()))
 check={'label':name,'target_rad_s':w,'equivalent_ground_m_s':w*.0325,'version':m['version'],'reason':m['reason'],'valid':m['full_rate_valid'],'active':m['full_rate_active'],'pi_updates':m['pi_updates'],'max_gap_us':m['max_sample_gap_us'],'max_encoder_age_us':m['max_encoder_age_us'],'missed_commit':m['missed_commit'],'latency':j.get('latency',[]),'metric':j['metric'],'motors':[]}
 passed=m['reason']==1 and m['cmp_errors']==0 and m['fault_seen']==0
 for motor in [1,2]:
  xs=[x for x in data if int(x['motor'])==motor and int(x['stage'])==2 and (int(x['us'])-m['profile_start'])*1e-6>1]
  t=[(int(x['us'])-m['profile_start'])*1e-6 for x in xs];a=[float(x['angle'])*j['setup']['direction'][motor-1] for x in xs]
  tm=statistics.mean(t);am=statistics.mean(a);fit=sum((u-tm)*(v-am) for u,v in zip(t,a))/sum((u-tm)**2 for u in t)
  peak=max(abs((j['lut_mv'][str(ch)][m[key][ch]]*.001-j['setup']['offset_V'][ch])*j['setup']['sense_gain'][motor-1]) for ch in [2*(motor-1),2*(motor-1)+1] for key in ['raw_min','raw_max'])
  limit_violations=sum(math.hypot(float(x['vd']),float(x['vq']))>float(x['vlimit'])+2e-6 for x in data if int(x['motor'])==motor)
  current_metric=next(q for q in j['metric'] if q['motor']==motor)
  check['motors'].append({'motor':motor,'fit_rad_s':fit,'AB_peak_A_nominal':peak,'limit_violations':limit_violations})
  passed=passed and abs(fit-w)<max(.5,abs(w)*.1) and peak<1.6 and not limit_violations and current_metric['iq_error_rms_A']<.15 and m['max_encoder_age_us'][motor-1]<1500 and m['max_sample_gap_us'][motor-1]<=800 and m['full_rate_valid'][motor-1]/max(1,m['full_rate_active'][motor-1])>=.999
 check['passed_progression_gate']=bool(passed);summary.append(check)
 (r/summary_name).write_text(json.dumps(summary,indent=2));print(json.dumps(check),flush=True)
 if not passed:raise SystemExit('STOPPED: progression gate failed: '+name)
print('ALL_BOUNDED_STEPS_COMPLETE',flush=True)
