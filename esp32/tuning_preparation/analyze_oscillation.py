"""Snapshot a growing UART200 CSV; quantify oscillations without touching the car."""
import argparse, csv, json
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
import numpy as np


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('session', type=Path)
    p.add_argument('--seconds', type=float, default=180)
    a = p.parse_args()
    # Only complete CSV records from a bounded tail; retain original device timestamps.
    with (a.session/'samples.csv').open(newline='', encoding='utf-8') as f:
        reader = csv.reader(f); names = next(reader)
        rows = [r for r in deque(reader, maxlen=int(a.seconds*210)) if len(r)==len(names)]
    x = np.asarray(rows, dtype=float)
    col = lambda k: x[:, names.index(k)]
    x = x[col('sample_us') >= col('sample_us')[-1]-a.seconds*1e6]
    out = a.session/('oscillation_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
    out.mkdir()
    with (out/'samples.csv').open('w', newline='', encoding='utf-8') as f:
        w=csv.writer(f); w.writerow(names); w.writerows(x)
    meta=json.loads((a.session/'metadata.json').read_text())
    meta.update(snapshot_source=str(a.session),snapshot_utc=datetime.now(timezone.utc).isoformat(),
                operator_context='User reported one light push shortly before this snapshot; exact contact onset and release uninstrumented.')
    (out/'metadata.json').write_text(json.dumps(meta,indent=2))
    t=(col('sample_us')-col('sample_us')[0])/1e6
    valid=(col('mode')==2)&((col('flags').astype(int)&7)==7)&(col('fault')==0)
    valid &= (col('imu_age_us')<=20000)&(col('encoder_age_us')<=1500)
    valid &= (col('current_age_a_us')<1000)&(col('current_age_b_us')<1000)
    def metrics(mask):
        z=x[mask]; c=lambda k:z[:,names.index(k)]
        if len(z)<200:return {'samples':len(z)}
        pitch=np.rad2deg(c('pitch')); speed=c('speed')
        dt=np.diff(c('sample_us'))/1e6
        result=dict(samples=len(z),duration_s=float(np.ptp(c('sample_us'))/1e6),
            pitch_mean_deg=float(np.mean(pitch)),pitch_std_deg=float(np.std(pitch)),
            pitch_range_deg=float(np.ptp(pitch)),speed_rms_m_s=float(np.sqrt(np.mean(speed**2))),
            saturation_fraction=float(np.mean(c('saturated')!=0)),
            missing_cycles=int(np.maximum(0,np.diff(c('cycle'))-1).sum()))
        # Spectra are only meaningful for contiguous, uniform, fixed-gain windows.
        if len(np.unique(c('revision')))==1 and np.max(dt)<.0075:
            fs=1/np.median(dt); n=len(z); f=np.fft.rfftfreq(n,1/fs)
            for key in ['pitch','target_pitch','speed','pitch_rate']:
                signal=c(key)-np.mean(c(key)); power=abs(np.fft.rfft(signal*np.hanning(n)))**2
                band=(f>=.1)&(f<=8); peak=np.flatnonzero(band)[np.argmax(power[band])]
                result[key+'_dominant_hz']=float(f[peak])
        return result
    reports=[]
    for start in np.arange(0,max(t[-1]-5,0),15):
        mask=(t>=start)&(t<start+15)&valid
        reports.append(dict(start_s=float(start),end_s=float(min(t[-1],start+15)),**metrics(mask)))
    # Align measured current with previous cycle, including motor A's forward sign.
    aligned=valid[1:]&valid[:-1]&(np.diff(col('cycle'))==1)
    err=np.stack([col('iq_a')[1:]+col('command_a')[:-1],col('iq_b')[1:]-col('command_b')[:-1]],axis=1)
    gains={k:float(col(k)[-1]) for k in names[-10:]}
    consistency=[]
    for horizon in [.05,.1,.5]:
        n=round(horizon/.005);residual=[];angle_change=[]
        for start in range(0,len(x)-n,n):
            stop=start+n
            if not np.all(valid[start:stop+1]) or not np.all(np.diff(col('cycle')[start:stop+1])==1):continue
            observed=col('pitch')[stop]-col('pitch')[start]
            q=col('pitch_rate')[start:stop+1];tt=col('sample_us')[start:stop+1]/1e6
            integral=float(np.sum((q[1:]+q[:-1])*.5*np.diff(tt)))
            residual.append(observed-integral);angle_change.append(observed)
        if residual:
            consistency.append(dict(horizon_s=horizon,blocks=len(residual),
                angle_change_rms_deg=float(np.rad2deg(np.sqrt(np.mean(np.square(angle_change))))),
                angle_minus_integrated_gyro_rms_deg=float(np.rad2deg(np.sqrt(np.mean(np.square(residual)))))))
    report=dict(source=str(a.session),device_start_us=float(col('sample_us')[0]),
        device_end_us=float(col('sample_us')[-1]),valid_fraction=float(np.mean(valid)),
        revisions=np.unique(col('revision')).astype(int).tolist(),gains=gains,
        whole=metrics(valid),windows=reports,
        previous_command_tracking_rms_A=np.sqrt(np.mean(err[aligned]**2,axis=0)).tolist(),
        attitude_rate_consistency=consistency,
        limitation='Push onset/release not instrumented; windows cannot establish settling time or causal parameter improvement. USB cable remains attached.')
    (out/'oscillation_report.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(dict(output=str(out),**report),indent=2))

if __name__=='__main__':main()
