"""Offline numeric checks; standard library only, no serial access."""
import csv,json,math,statistics,sys
from pathlib import Path
r=Path(__file__).resolve().parent
for label in sys.argv[1:]:
 j=json.loads((r/(label+'.json')).read_text());m=j['meta']
 rows=list(csv.DictReader((r/(label+'.csv')).open()))
 rows=[{k:float(v) for k,v in x.items()} for x in rows]
 group={}
 for x in rows:group.setdefault(int(x.get('motor',1)),[]).append(x)
 result={'label':label,'meta':m,'motors':{}}
 for motor,xs in group.items():
  active=[x for x in xs if int(x['flags'])&63==63]
  plateau=[x for x in active if x['stage']==2 and (x['us']-m['profile_start'])*1e-6>(1.0 if m['version']>=13 else .65)]
  if m['mode']==3:plateau=[x for x in active if .14<(x['us']-m['profile_start'])*1e-6<.19]
  def avg(name,seq=plateau):return statistics.mean(x[name] for x in seq) if seq else None
  def sd(name,seq=plateau):return statistics.stdev(x[name] for x in seq) if len(seq)>1 else None
  def rms(name,ref,seq=plateau):return math.sqrt(statistics.mean((x[name]-x[ref])**2 for x in seq)) if seq else None
  fit=None
  if len(plateau)>2 and m['mode']==6:
   t=[(x['us']-m['profile_start'])*1e-6 for x in plateau]
   a=[x.get('angle',x.get('left')) for x in plateau];tm=statistics.mean(t);am=statistics.mean(a)
   direction=j.get('setup',{}).get('direction',[-1,1])[motor-1]
   fit=direction*sum((u-tm)*(v-am) for u,v in zip(t,a))/sum((u-tm)**2 for u in t)
  lut=j.get('lut_mv',{});buslut=lut.get('4') if m['version']>=13 else lut.get('2')
  buses=[buslut[int(x['raw_bus'])]*.003 for x in xs] if buslut else [x['bus_V'] for x in xs if 'bus_V' in x]
  margins=[min(x['a0']-x['ca']-160,8000-x['ca']-x['a1']-160,x['b0']-x['cb']-160,8000-x['cb']-x['b1']-160)/80 for x in active]
  raw_max=None
  if m['version']>=13:
   raw_max=max(abs((lut[str(ch)][m[key][ch]]*.001-j['setup']['offset_V'][ch])*j['setup']['sense_gain'][motor-1]) for ch in [2*(motor-1),2*(motor-1)+1] for key in ['raw_min','raw_max'])
  stale=[x['latch_us']-x['applied_sample_us'] for x in xs if x['applied_sample_us']>0]
  result['motors'][motor]={'rows':len(xs),'active_saved':len(active),'speed_fit_rad_s':fit,'speed_mean':avg('speed'),'speed_std':sd('speed'),'id_mean_A':avg('id'),'id_std_A':sd('id'),'iq_error_rms_A':rms('iq','iq_ref'),'id_error_rms_A':rms('id','id_ref'),'voltage_peak_V':max(math.hypot(x['vd'],x['vq']) for x in xs),'bus_range_V':[min(buses),max(buses)] if buses else None,'limit_range_V':[min(x['vlimit'] for x in xs),max(x['vlimit'] for x in xs)],'limit_violations':sum(math.hypot(x['vd'],x['vq'])>x['vlimit']+2e-6 for x in active),'max_calc_age_us':max(x['calc_us']-x['us'] for x in xs),'minimum_saved_valid_window_margin_us':min(margins) if margins else None,'fullrate_AB_peak_A_nominal':raw_max,'saved_sample_to_latch_range_us':[min(stale),max(stale)] if stale else None,'plateau_saved_samples':len(plateau)}
 (r/(label+'_analysis.json')).write_text(json.dumps(result,indent=2))
 print(json.dumps(result))
