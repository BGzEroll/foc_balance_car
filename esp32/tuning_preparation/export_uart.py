"""Preserve serial captures with actual gains, metrics and transition-window labels."""
import argparse,csv,io,json,math,re,time
from pathlib import Path
from statistics import mean,pstdev

def export(folder):
 text=(folder/'uart.log').read_text(encoding='utf-8')
 tokens=re.findall(r'GAINS schema=[^\r\n]+|CSV_BEGIN [^\r\n]+\s+t_us,[\s\S]*?CSV_END',text)
 gains={};revision=None;settling=0;catalog=[];index=0
 for token in tokens:
  if token.startswith('GAINS '):
   fields=dict(re.findall(r'(\w+)=([^ ]+)',token));new=int(fields['revision'])
   if revision is not None and revision!=new:settling=2
   revision=new;gains={k:float(v) for k,v in fields.items() if k not in ('schema','revision')}
   continue
  index+=1
  data=token[token.index('t_us,'):token.rindex('CSV_END')].strip()
  rows=list(csv.DictReader(io.StringIO(data)))
  if len(rows)!=400 or revision is None:continue
  try:values=[{k:float(v) for k,v in r.items()} for r in rows]
  except (TypeError,ValueError):continue
  if any(not math.isfinite(v) for r in values for v in r.values()):continue
  stem=folder/f'capture_{index:04}_r{revision}'
  pitch=[math.degrees(r['pitch']) for r in values]
  report=dict(capture=index,revision=revision,transition_window=settling>0,gains=gains,
   rows=len(rows),duration_s=(values[-1]['t_us']-values[0]['t_us'])/1e6,
   recorded_hz=1e6/mean(values[i]['t_us']-values[i-1]['t_us'] for i in range(1,len(values))),
   control_period_s=.005,pitch_mean_deg=mean(pitch),pitch_rms_deg=math.sqrt(mean(x*x for x in pitch)),
   pitch_std_deg=pstdev(pitch),pitch_peak_deg=max(map(abs,pitch)),
   speed_rms_m_s=math.sqrt(mean(r['speed']**2 for r in values)),
   command_rms_A=[math.sqrt(mean(r[k]**2 for r in values)) for k in ['cmd_a','cmd_b']],
   id_rms_A=[math.sqrt(mean(r[k]**2 for r in values)) for k in ['id_a','id_b']],
   saturation_fraction=mean(r['saturated'] for r in values),
   source='Raw UART rolling capture; PID runs at 200Hz, retained rows at 40Hz. First two snapshots after a gain revision are excluded from comparisons.')
  settling=max(0,settling-1)
  if not stem.with_suffix('.json').exists():
   stem.with_suffix('.csv').write_text(data+'\n',encoding='utf-8')
   stem.with_suffix('.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
   print(json.dumps({k:report[k] for k in ['capture','revision','transition_window','pitch_rms_deg','pitch_std_deg','command_rms_A']}),flush=True)
  catalog.append(report)
 if catalog:
  (folder/'catalog.json').write_text(json.dumps(catalog,indent=2),encoding='utf-8')
 return len(catalog)

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('folder',type=Path);p.add_argument('--watch',action='store_true');a=p.parse_args()
 while True:
  export(a.folder)
  if not a.watch:break
  time.sleep(3)
