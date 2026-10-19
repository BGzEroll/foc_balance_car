from pathlib import Path
import csv,json,math,statistics as st,sys
r=Path(__file__).resolve().parent;name=sys.argv[1]
j=json.loads((r/f'{name}.json').read_text());m=j['meta'];x=[{k:float(v) for k,v in z.items()} for z in csv.DictReader((r/f'{name}.csv').open())]
avg=lambda v:sum(v)/len(v) if v else None
def stats(v):return dict(mean=avg(v),std=st.pstdev(v) if v else None,min=min(v) if v else None,max=max(v) if v else None)
for z in x:
 z['t']=(z['us']-m['profile_start'])*1e-6
 z['ia']=(j['lut_mv']['0'][int(z['raw_a'])]*.001-m['offset_V'][0])*m['gain_A_per_V']
 z['ib']=(j['lut_mv']['1'][int(z['raw_b'])]*.001-m['offset_V'][1])*m['gain_A_per_V']
 z['bus']=j['lut_mv']['2'][int(z['raw_bus'])]*.003
 c=(z['ca']+z['cb']+z['cc'])/3
 z['va']=(z['ca']-c)/(m['period_ticks']/2)*z['bus'];z['vb']=(z['cb']-c)/(m['period_ticks']/2)*z['bus']
out={'label':name,'reason':m['reason'],'rows':len(x),'valid_fraction':m['full_rate_valid']/m['full_rate_active'] if m['full_rate_active'] else 0,
 'current_a':stats([z['ia'] for z in x]),'current_b':stats([z['ib'] for z in x]),
 'calc_age_us':stats([z['calc_us']-z['us'] for z in x]),
 'applied_age_us':stats([z['latch_us']-z['applied_sample_us'] for z in x if z['applied_sample_us']]),
 'a_start_us':stats([z['a0']*100/m['period_ticks'] for z in x]),'b_end_us':stats([z['b1']*100/m['period_ticks'] for z in x]),
 'sample_gap_us':stats([b['us']-a['us'] for a,b in zip(x,x[1:])]),'stages':[]}
for stage in sorted(set(z['stage'] for z in x)):
 ss=[z for z in x if z['stage']==stage];end=max(z['t'] for z in ss);ss=[z for z in ss if z['t']>end-.06]
 out['stages'].append({'stage':stage,'t_end':end,**{k:stats([z[k] for z in ss]) for k in ['ia','ib','left','speed','id','iq','id_ref','iq_ref','vd','vq','va','vb']}})
if m['mode']==1 and m['reason']==1:
 alpha=out['stages'][0]['left']['mean'];beta=out['stages'][1]['left']['mean'];last=out['stages'][2]['left']['mean']
 out['encoder_beta_minus_alpha']=beta-alpha;out['expected_encoder_beta_minus_alpha']=-math.pi/14
 out['encoder_return_error']=last-alpha;out['provisional_zero']=(-7*last)%(2*math.pi)
 # Check relative signs/ratios; this does not calibrate the absolute ampere scale.
 a,b,c=out['stages'];out['polarity_pass']=a['ia']['mean']>.08 and a['ib']['mean']<-.04 and b['ib']['mean']>.08
 out['encoder_pass']=abs(beta-alpha+math.pi/14)<.06 and abs(last-alpha)<.04
(r/f'{name}_analysis.json').write_text(json.dumps(out,indent=2))
print(json.dumps(out,indent=2))
