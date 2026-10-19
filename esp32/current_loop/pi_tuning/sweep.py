"""Finite v18 bench batches. Any failed protection/quality gate stops the batch."""
from pathlib import Path
import argparse,csv,json,math,statistics,subprocess,sys,time
r=Path(__file__).resolve().parent
GAINS={'base':(.060318579,226.194672),'soft':(.030159290,113.097336),'mid':(.045238934,169.646004),'lowki':(.060318579,113.097336)}

def analyze(name):
 j=json.loads((r/(name+'.json')).read_text());m=j['meta'];rows=[{k:float(v) for k,v in x.items()} for x in csv.DictReader((r/(name+'.csv')).open())]
 z={'name':name,'kp':m['kp'],'ki':m['ki'],'meta':m,'metric':j['metric'],'motors':[]}
 for motor in [1,2]:
  xs=[x for x in rows if x['motor']==motor and int(x['flags'])&63==63];out={'motor':motor}
  bus=[j['lut_mv']['4'][int(x['raw_bus'])]*.003 for x in xs]
  out.update(bus_range_V=[min(bus),max(bus)],peak_saved_speed=max(abs(x['speed']) for x in xs),limit_violations=sum(math.hypot(x['vd'],x['vq'])>x['vlimit']+2e-6 for x in xs))
  out['saved_saturation_fraction']=sum(math.hypot(x['vd'],x['vq'])>=x['vlimit']-1e-5 for x in xs)/len(xs)
  out['raw_AB_peak_A']=max(abs((j['lut_mv'][str(ch)][m[key][ch]]*.001-j['setup']['offset_V'][ch])*j['setup']['sense_gain'][motor-1]) for ch in [2*(motor-1),2*(motor-1)+1] for key in ['raw_min','raw_max'])
  if m['mode']==3:
   amp=max(x['id_ref'] for x in xs);edge=[x for x in xs if .1<=(x['us']-m['profile_start'])*1e-6<.14]
   out['step_10_90_ms']=None
   a=next((x for x in edge if x['id']>=amp*.55),None);b=next((x for x in edge if x['id']>=amp*.95),None)
   if a and b:out['step_10_90_ms']=(b['us']-a['us'])*.001
   out['saved_40ms_peak_above_ref_A']=max(x['id'] for x in edge)-amp
   out['saved_10ms_peak_above_ref_A']=max(x['id'] for x in edge if (x['us']-m['profile_start'])*1e-6<.11)-amp
   out['early_step_rmse_A']=math.sqrt(statistics.mean((x['id']-amp)**2 for x in edge if (x['us']-m['profile_start'])*1e-6<.11))
  if m['mode']==6:
   q=[x for x in xs if x['stage']==2 and (x['us']-m['profile_start'])*1e-6>1]
   ts=[(x['us']-m['profile_start'])*1e-6 for x in q];ys=[x['angle']*j['setup']['direction'][motor-1] for x in q];tm=statistics.mean(ts);ym=statistics.mean(ys)
   out['speed_fit_rad_s']=sum((t-tm)*(y-ym) for t,y in zip(ts,ys))/sum((t-tm)**2 for t in ts)
  if m['mode'] in [4,5]:
   q=[x for x in xs if any(lo<((x['us']-m['profile_start'])*1e-6)<hi for lo,hi in [(.045,.059),(.105,.119),(.165,.179),(.185,.199)])]
   out['saved_q_plateau_rmse_A']=math.sqrt(statistics.mean((x['iq']-x['iq_ref'])**2 for x in q))
   out['saved_q_positive_mean_A']=statistics.mean(x['iq'] for x in q if x['iq_ref']>0)
   out['saved_q_negative_mean_A']=statistics.mean(x['iq'] for x in q if x['iq_ref']<0)
   out['saved_q_plateau_samples']=len(q)
  z['motors'].append(out)
 z['progression_pass']=m['reason']==1 and not m['cmp_errors'] and not m['fault_seen'] and not any(m['cross']) and all(v/max(1,n)>=.995 for v,n in zip(m['full_rate_valid'],m['full_rate_active'])) and max(m['max_sample_gap_us'])<=800 and max(m['max_encoder_age_us'])<1500 and all(not x['limit_violations'] and x['raw_AB_peak_A']<1.8 for x in z['motors'])
 if m['mode']==6:z['progression_pass'] &= all(abs(x['speed_fit_rad_s']-j['setup']['speed_target'])<max(.5,.1*abs(j['setup']['speed_target'])) for x in z['motors']) and all(x['iq_error_rms_A']<.15 for x in j['metric'])
 (r/(name+'_analysis.json')).write_text(json.dumps(z,indent=2));return z

def run(name,gain,command,limit=(.015,.12)):
 print('BEGIN '+name,flush=True)
 cmd=[sys.executable,str(r/'run.py'),name,'--mask','3','--zero','1','-1','5.330672807','--zero','2','1','2.164932114','--gains',*map(str,(*GAINS[gain],0)),'--limit',*map(str,limit),'--command',command]
 with (r/(name+'_runner.log')).open('w') as f:ret=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT).returncode
 if ret:raise RuntimeError('Command stopped/rejected: '+name)
 z=analyze(name);print(json.dumps(z),flush=True)
 if not z['progression_pass']:raise RuntimeError('Quality gate failed: '+name)
 return z

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('stage',choices=['screen','speed','current']);p.add_argument('--gains',nargs='+',choices=list(GAINS),default=list(GAINS));p.add_argument('--tag',default='');p.add_argument('--repeats',type=int,default=2);a=p.parse_args();zs=[]
 try:
  if a.stage=='screen':
   for rep in range(a.repeats):
    for gain in (a.gains if rep%2==0 else list(reversed(a.gains))):zs.append(run(f'{gain}_d030_{a.tag}r{rep+1}',gain,'DTEST 0.30 350 2'))
  elif a.stage=='speed':
   for speed in [5,10,27.5,-27.5]:
    for gain in a.gains:zs.append(run(f'{gain}_w{speed:g}'.replace('-','m').replace('.','p'),gain,f'SPEED {speed} 4000 25',(.04,.4)))
  else:
   for gain in a.gains:
    for amp in [.1,.5]:zs.append(run(f'{gain}_d{amp:g}'.replace('.','p'),gain,f'DTEST {amp} 350 2'))
    for amp in [.1,.2]:zs.append(run(f'{gain}_q{amp:g}'.replace('.','p'),gain,f'QZERO {amp} 250 2'))
 finally:(r/(a.stage+'_'+a.tag+'summary.json')).write_text(json.dumps(zs,indent=2))
 print('BATCH_COMPLETE '+a.stage,flush=True)
