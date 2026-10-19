"""Offline inspection of static alpha/beta voltage vectors and alignment."""
import csv,json,math,statistics,sys
from pathlib import Path
r=Path(__file__).resolve().parent;j=json.loads((r/(sys.argv[1]+'.json')).read_text());m=j['meta'];rows=list(csv.DictReader((r/(sys.argv[1]+'.csv')).open()))
v=[{k:float(x) for k,x in row.items()} for row in rows]
results=[]
for stage,(lo,hi) in enumerate([(.22,.30),(.52,.60),(.82,.90)]):
 xs=[x for x in v if lo<(x['us']-m['profile_start'])*1e-6<hi]
 if not xs:continue
 motor=int(xs[0]['motor']);off=j['setup']['offset_V'];gain=j['setup']['sense_gain'][motor-1];lut=j['lut_mv'];ia=[];ib=[]
 for x in xs:
  ia.append((lut[str(2*(motor-1))][int(x['raw_a'])]*.001-off[2*(motor-1)])*gain)
  ib.append((lut[str(2*(motor-1)+1)][int(x['raw_b'])]*.001-off[2*(motor-1)+1])*gain)
 results.append({'stage':stage,'angle_mean':statistics.mean(x['angle'] for x in xs),'angle_range':[min(x['angle'] for x in xs),max(x['angle'] for x in xs)],'ia':statistics.mean(ia),'ib':statistics.mean(ib),'alpha':statistics.mean(ia),'beta':statistics.mean([(a+2*b)/math.sqrt(3) for a,b in zip(ia,ib)]),'id':statistics.mean(x['id'] for x in xs),'iq':statistics.mean(x['iq'] for x in xs),'rows':len(xs)})
out={'label':sys.argv[1],'reason':m['reason'],'stages':results}
if len(results)==3:
 diff=results[1]['angle_mean']-results[0]['angle_mean'];direction=1 if diff>0 else -1
 out.update(direction=direction,mechanical_alpha_to_beta=diff,expected_magnitude=math.pi/14,zero_at_final_alpha=(direction*7*results[-1]['angle_mean'])%(2*math.pi))
(r/(sys.argv[1]+'_calibration.json')).write_text(json.dumps(out,indent=2))
print(json.dumps(out))
