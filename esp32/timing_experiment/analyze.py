from pathlib import Path
import json

root=Path(__file__).parent
all_results=[]
for path in sorted(root.glob('p*_yield*.json')):
 data=json.loads(path.read_text(encoding='utf-8'))
 frames=data['frames']
 steady=[f for f in frames if f['time_us']>=8000000]
 if len(steady)<2:steady=frames
 a,b=steady[0],steady[-1]
 seconds=(b['time_us']-a['time_us'])/1e6
 result={'label':data['label'],'sha256':data['sha256'],'failures':data['failure_markers'],
 'frames':len(frames),'capture_s':data['duration_s'],
 'steady_duration_s':seconds,
 'foc_hz':(b['foc']-a['foc'])/seconds,
 'adc_hz':[(y-x)/seconds for x,y in zip(a['samples'],b['samples'])],
 'idle_per_s':(b['idle']-a['idle'])/seconds,
 'late_half_fraction':(b['late_half']-a['late_half'])/(b['events']-a['events']),
 'ready_frames':sum(f['ready'] for f in frames),
 'sensor_age_observed_max_us':[max(f['sensor_age_us'][i] for f in steady) for i in range(3)],
 'last':b}
 print(json.dumps({k:v for k,v in result.items() if k!='last'},ensure_ascii=False))
 all_results.append(result)
(root/'summary.json').write_text(json.dumps(all_results,ensure_ascii=False,indent=2),encoding='utf-8')
