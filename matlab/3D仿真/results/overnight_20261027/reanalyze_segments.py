import csv
import datetime as dt
import json
import math
import pathlib
import statistics as st

p = pathlib.Path(__file__).resolve().parent
overview = [{k: float(v) for k,v in r.items()} for r in csv.DictReader((p/'overview.csv').open(encoding='utf-8'))]
yaw = [{k: float(v) for k,v in r.items()} for r in csv.DictReader((p/'yaw_1hz.csv').open(encoding='utf-8'))]
stop = 10180.151657
anchor = dt.datetime.fromisoformat('2026-10-27T03:08:18+08:00')
deg = 180/math.pi
def wall(s): return (anchor+dt.timedelta(seconds=s-stop)).isoformat(timespec='seconds')

# Sustained voltage restorations; the user confirmed the first involved handling.
steps = [next(r['device_s'] for r in overview if r['elapsed_min']>7 and r['bus_V']>8.18),
         next(r['device_s'] for r in overview if r['elapsed_min']>60 and r['bus_V']>8.18)]
windows = [(steps[0]-120,steps[0]+120)]
begin = overview[0]['device_s']
end = stop-60
bounds = [(begin,windows[0][0]), (windows[0][1],end)]

# Distinguish subsequent autonomous movement from an approximately stationary test.
moving = None
for i,r in enumerate(overview):
    following = overview[i:i+12]
    if r['device_s']>bounds[1][0] and len(following)==12 and all(abs(v['distance_cm'])>1 for v in following):
        moving = r['device_s']; break
bounds = bounds[:1]+[(bounds[1][0],moving),(moving,end)]
segments = []
curves = []
for index,(lo,hi) in enumerate(bounds,1):
    rows = [r for r in yaw if lo <= r['time_us']/1e6 < hi]
    first = [r for r in rows if r['time_us']/1e6 < lo+60]
    last = [r for r in rows if r['time_us']/1e6 >= hi-60]
    cb = st.mean(r['corrected_yaw'] for r in first)
    eb = st.mean(r['encoder_yaw'] for r in first)
    rb = st.mean(r['raw_yaw'] for r in first)
    dev = [(r['corrected_yaw']-cb)*deg for r in rows]
    sorted_abs = sorted(abs(v) for v in dev)
    duration = (rows[-1]['time_us']-rows[0]['time_us'])/1e6
    learned = sum(((rows[i]['time_us']-rows[i-1]['time_us'])/1e6) for i in range(1,len(rows)) if int(rows[i]['flags'])&4)
    s = dict(segment=index,start_beijing=wall(lo),end_beijing=wall(hi),start_device_s=lo,end_device_s=hi,
             duration_min=duration/60,condition='autonomous forward/backward displacement >1 cm' if index==len(bounds) else 'forward/backward odometry remains within 1 cm; does not prove no rotation',
             estimator_first_last_60s_mean_change_deg=(st.mean(r['corrected_yaw'] for r in last)-cb)*deg,
             encoder_first_last_60s_mean_change_deg=(st.mean(r['encoder_yaw'] for r in last)-eb)*deg,
             estimator_abs_deviation_p95_deg=sorted_abs[int(.95*(len(sorted_abs)-1))],
             estimator_deviation_range_deg=[min(dev),max(dev)],learning_fraction=learned/duration)
    segments.append(s)
    for r in rows:
        curves.append([index,(r['time_us']/1e6-begin)/60,(r['corrected_yaw']-cb)*deg,(r['encoder_yaw']-eb)*deg,(r['raw_yaw']-rb)*deg,r['bias']*deg,1 if int(r['flags'])&4 else 0])
with (p/'segmented_yaw.csv').open('w',newline='',encoding='utf-8') as f:
    w=csv.writer(f);w.writerow(['segment','elapsed_min','estimated_yaw_deg','encoder_yaw_deg','raw_integral_deg','bias_deg_s','learning']);w.writerows(curves)
summary=json.loads((p/'summary.json').read_text(encoding='utf-8'))
old_keys=[k for k in summary if k.startswith(('yaw_','pre_online_bias','encoder_heading','bias_range','bias_learning'))]
if old_keys:
    summary['previous_unsplit_metrics_not_valid_for_natural_drift']={k:summary.pop(k) for k in old_keys}
summary['segmented_statistics_baseline']='separate first-60s mean in each segment; endpoint is last-60s mean'
summary['exclusion_window_policy']='120 seconds before and after the user-confirmed 00:39 handling event only; conservative boundary, not measured handling duration'
summary['exclusions']=[dict(start_beijing=wall(lo),end_beijing=wall(hi),start_device_s=lo,end_device_s=hi,
    reason='user-confirmed handling while connecting power') for lo,hi in windows]
summary['retained_supply_event']=dict(time_beijing=wall(steps[1]),device_s=steps[1],reason='user unplugged/replugged charger supply side only; no handling of car; data retained without yaw rebase')
summary['autonomous_displacement_onset_beijing']=wall(moving)
summary['segments']=segments
(p/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps({'exclusions':summary['exclusions'],'movement_onset':wall(moving),'segments':segments},ensure_ascii=False,indent=2))
