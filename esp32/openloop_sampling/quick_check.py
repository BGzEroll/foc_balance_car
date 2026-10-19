from pathlib import Path
import csv,json,math,statistics as st,sys
r=Path(__file__).resolve().parent
def unwrap(x):
 out=[x[0]]
 for i in range(1,len(x)):out.append(out[-1]+(x[i]-x[i-1]+math.pi)%(2*math.pi)-math.pi)
 return out
for label in sys.argv[1:]:
 j=json.loads((r/f'{label}.json').read_text());m=j['meta'];lut=j['lut_mv']
 rows=[{k:float(v) for k,v in x.items()} for x in csv.DictReader((r/f'{label}.csv').open())]
 period=m.get('period_ticks',500)
 ma=[min(x['a0']-x['ca'],period-x['ca']-x['a1'])/10 for x in rows]
 mb=[min(x['b0']-x['cb'],period-x['cb']-x['b1'])/10 for x in rows]
 on=[i for i,x in enumerate(rows) if int(x['flags'])&1]
 valid=[i for i in on if ma[i]>=2 and mb[i]>=2 and int(rows[i]['flags'])&38==38]
 currents=[[ (lut[str(c)][int(x['raw_a' if c==0 else 'raw_b'])]/1000-m['offset_V'][c])*4 for x in rows] for c in range(2)]
 left=unwrap([x['left'] for x in rows]);right=unwrap([x['right'] for x in rows]);t=[(x['us']-rows[0]['us'])/1e6 for x in rows]
 stable=[i for i in on if t[i]>.5];stable=stable or list(range(len(rows)))
 mt=st.mean(t[i] for i in stable);ml=st.mean(left[i] for i in stable)
 speed=sum((t[i]-mt)*(left[i]-ml) for i in stable)/sum((t[i]-mt)**2 for i in stable)
 s={'label':label,'rows':len(rows),'active':len(on),'valid':len(valid),'valid_pct':100*len(valid)/max(len(on),1),'margins_min_us':[min(ma),min(mb)],'margins_median_us':[st.median(ma),st.median(mb)],'pair_median_us':st.median((x['b1']-x['a0'])/10 for x in rows),'left_delta_rad':left[-1]-left[0],'right_delta_rad':right[-1]-right[0],'speed_fit_rad_s':speed,'current_abs_max_A':[max(map(abs,a)) for a in currents],'current_std_A':[st.stdev(a) for a in currents],'reason':m['reason'],'late_commit':m['late_commit']}
 (r/f'{label}_quick.json').write_text(json.dumps(s,indent=2));print(json.dumps(s))
