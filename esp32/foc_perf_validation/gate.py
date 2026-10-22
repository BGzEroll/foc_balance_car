"""Offline escalation gate; never opens USB or commands hardware."""
from pathlib import Path
import csv,json,sys,math
p=Path(__file__).resolve().parent/sys.argv[1];p=p.with_suffix('.json')
d=json.loads(p.read_text());meta=d['META'][-1];status=d['STATUS'][-1]
assert not status['fault'] and not status['running'] and not status['requested'],status
rows=list(csv.DictReader(p.with_suffix('.csv').open()));result=[]
for m in d['METRIC']:
 if not meta['mask']&(1<<(m['motor']-1)):continue
 assert m['active']>0 and m['valid']/m['active']>=.995,m
 assert m['max_gap_us']<=1000 and m['n']>0,m
 rr=[r for r in rows if int(r['motor'])==m['motor'] and 1<=float(r['us'])*1e-6<meta['duration_ms']*.001-.75]
 assert len(rr)>20
 if meta['kind']==2:
  ts=[float(r['us'])*1e-6 for r in rr];ys=[float(r['angle']) for r in rr]
  tx=sum(ts)/len(ts);ay=sum(ys)/len(ys)
  speed=status['direction'][m['motor']-1]*sum((t-tx)*(y-ay) for t,y in zip(ts,ys))/sum((t-tx)**2 for t in ts)
  assert abs(speed-meta['value'])<=max(.4,abs(meta['value'])*.03),(speed,meta)
 else:
  speed=None;assert abs(m['id_mean']-meta['value'])<.03,(m,meta)
 assert all(math.hypot(float(r['vd']),float(r['vq']))<=float(r['limit'])+1e-5 for r in rows if int(r['motor'])==m['motor'])
 result.append({'motor':m['motor'],'speed_rad_s':speed,'id_mean':m['id_mean'],'valid':m['valid'],'active':m['active'],'max_PI_gap_us':m['max_gap_us'],'observed_voltage_max_V':m['voltage_max']})
out={'label':p.stem,'result':'PASS_BENCH_GATE','motors':result}
p.with_name(p.stem+'_gate.json').write_text(json.dumps(out,indent=2));print(json.dumps(out))
