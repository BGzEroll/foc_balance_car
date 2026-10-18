from pathlib import Path
import json
r=Path(__file__).parent
out=[]
for file in r.glob('*.json'):
 if file.name=='summary.json':continue
 d=json.loads(file.read_text(encoding='utf-8'))
 if 'frames' not in d:continue
 frames=[f for f in d['frames'] if f['events'] and f['time_us']>10000000]
 if len(frames)<2:continue
 a,b=frames[0],frames[-1]
 dt=(b['time_us']-a['time_us'])/1e6
 events=b['events']-a['events']
 result={'label':d['label'],'sha256':d['sha256'],'duration_s':d['duration_s'],'failure_markers':d['failure_markers'],
 'steady_s':dt,'foc_hz':(b['foc']-a['foc'])/dt,'event_hz':events/dt,
 'adc_hz':[(y-x)/dt for x,y in zip(a['samples'],b['samples'])],
 'late_fraction':(b['late_atomic']-a['late_atomic'])/events,
 'gaps':b['gaps']-a['gaps'],'ready_all_steady':all(f['ready'] for f in frames),
 'sensor_age_max_us':[max(f['sensor_age_us'][i] for f in frames) for i in range(3)],
 'last':b}
 if 'rejected' in a:
  result['accepted_pair_hz']=[((b['samples'][2*i]-a['samples'][2*i])-(b['rejected'][i]-a['rejected'][i]))/dt for i in range(2)]
  result['rejected_total']=b['rejected']
 out.append(result)
 print(json.dumps({k:v for k,v in result.items() if k!='last'},ensure_ascii=False))
(r/'summary.json').write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf-8',newline='\n')
