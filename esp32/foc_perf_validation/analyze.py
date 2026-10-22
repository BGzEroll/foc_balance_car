"""Read-only analysis of saved new-core and v18 bench data (standard library)."""
from pathlib import Path
import json,csv,math
root=Path(__file__).resolve().parent/'evidence'
def slope(rows):
 ts=[float(r['us'])*1e-6 for r in rows];ys=[float(r['angle']) for r in rows]
 if len(ts)<3:return None
 tx=sum(ts)/len(ts);ay=sum(ys)/len(ys)
 return sum((t-tx)*(y-ay) for t,y in zip(ts,ys))/sum((t-tx)**2 for t in ts)
summary=[]
for path in sorted(root.glob('*.json')):
 d=json.loads(path.read_text());new='META' in d;ref='meta' in d and 'metric' in d
 if not(new or ref) or not path.with_suffix('.csv').exists():continue
 meta=d['META'][-1] if new else d['meta'];meta=meta[-1] if isinstance(meta,list) else meta
 metrics=d['METRIC'] if new else d['metric'];status=(d['STATUS'] if new else d['status'])[-1]
 rows=list(csv.DictReader(path.with_suffix('.csv').open()))
 if rows:
  first=min(float(r['us']) for r in rows)
  # Reference CSV may contain absolute acquisition timestamps.
  if first>meta['duration_ms']*1000:
   start=meta.get('profile_start',first)
   for r in rows:r['us']=str(float(r['us'])-start)
 for m in metrics:
  motor=m['motor'];rr=[r for r in rows if int(r['motor'])==motor and 1<=float(r['us'])*1e-6<(meta['duration_ms']*.001-.75)]
  v=slope(rr)
  if v is not None:
   if new:v*=status['direction'][motor-1]
   elif motor==1:v=-v
  sample_eq=[float(r['iq'])-float(r['target_q'] if new else r['iq_ref']) for r in rr]
  sample_ed=[float(r['id'])-float(r['target_d'] if new else r['id_ref']) for r in rr]
  summary.append({'case':path.stem,'motor':motor,'fault':status.get('fault',meta.get('reason',1)!=1),
   'sampled_iq_error_rms_A':math.sqrt(sum(x*x for x in sample_eq)/len(sample_eq)) if sample_eq else None,
   'sampled_id_error_rms_A':math.sqrt(sum(x*x for x in sample_ed)/len(sample_ed)) if sample_ed else None,
   'speed_rad_s':v,'id_mean':m.get('id_mean',m.get('id_mean_A')),
   'iq_error_rms_A':m.get('iq_error_rms',m.get('iq_error_rms_A')),
   'voltage_max_V':m.get('voltage_max',max((math.hypot(float(r['vd']),float(r['vq'])) for r in rr),default=None)),
   'max_gap_us':m.get('max_gap_us',meta.get('max_sample_gap_us',[None,None])[motor-1]),
   'active':m.get('active',meta.get('full_rate_active',[None,None])[motor-1]),
   'valid':m.get('valid',meta.get('full_rate_valid',[None,None])[motor-1]),
   'plateau_updates':m['n'],'rows':len(rr)})
(root/'analysis_summary.json').write_text(json.dumps(summary,indent=2))
for s in summary:
 if s['case'].startswith(('v18_settings','wide','reference','final')):print(json.dumps(s))
