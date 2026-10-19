"""Offline evidence index. Includes failed quality gates rather than hiding them."""
from pathlib import Path
import json,csv,statistics
r=Path(__file__).resolve().parent
js=[json.loads(p.read_text()) for p in sorted(r.glob('*_analysis.json'))]
out={'trials':len(js),'quality_gate_failures':[j['name'] for j in js if not j['progression_pass']],
 'protection_failures':[j['name'] for j in js if j['meta']['reason']!=1 or j['meta']['fault_seen']],
 'bus_range_V':[min(x['bus_range_V'][0] for j in js for x in j['motors']),max(x['bus_range_V'][1] for j in js for x in j['motors'])],
 'max_raw_AB_A':max(x['raw_AB_peak_A'] for j in js for x in j['motors']),
 'max_encoder_age_us':max(max(j['meta']['max_encoder_age_us']) for j in js),
 'max_pi_gap_us':max(max(j['meta']['max_sample_gap_us']) for j in js),
 'valid_groups':[sum(j['meta']['full_rate_valid'][m] for j in js) for m in [0,1]],
 'active_groups':[sum(j['meta']['full_rate_active'][m] for j in js) for m in [0,1]],
 'saved_voltage_limit_violations':sum(x['limit_violations'] for j in js for x in j['motors']),
 'screen':{},'speed':[],'current':[]}
for key in ['base','soft','mid','lowki']:
 q=[j for j in js if j['name'].startswith(key+'_d030_')]
 out['screen'][key]={'n':len(q),'quality_passes':sum(j['progression_pass'] for j in q),
  'kp':q[0]['kp'],'ki':q[0]['ki'],
  'id_std_mA_range':[[min(j['metric'][m]['id_std_A']*1000 for j in q),max(j['metric'][m]['id_std_A']*1000 for j in q)] for m in [0,1]],
  'id_mean_A_range':[[min(j['metric'][m]['id_mean_A'] for j in q),max(j['metric'][m]['id_mean_A'] for j in q)] for m in [0,1]],
  'rise_10_90_ms_range':[[min(j['motors'][m]['step_10_90_ms'] for j in q),max(j['motors'][m]['step_10_90_ms'] for j in q)] for m in [0,1]]}
for j in js:
 if j['meta']['mode']==6:out['speed'].append({'name':j['name'],'speed_fit':[x['speed_fit_rad_s'] for x in j['motors']],'iq_rms_mA':[1000*x['iq_error_rms_A'] for x in j['metric']],'saturation_fraction':[x['saturation_fraction'] for x in j['metric']]})
 elif '_d030_' not in j['name']:out['current'].append({'name':j['name'],'metric':j['metric'],'motors':j['motors']})
(r/'evidence_summary.json').write_text(json.dumps(out,indent=2))
print(json.dumps(out,indent=2))
