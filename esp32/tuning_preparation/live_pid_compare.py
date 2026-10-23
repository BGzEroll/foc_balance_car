"""Small live PID comparisons on an already balancing car. Never CAL/BALANCE/STOP."""
import argparse, json, math, re, time
from datetime import datetime, timezone
from pathlib import Path
import numpy as np
from uart200 import Link, FIELDS2, HERE

NAMES=FIELDS2[-10:]
def metrics(rows):
    a=np.asarray(rows,dtype=float);c=lambda k:a[:,FIELDS2.index(k)]
    assert len(a)>=3000,'Insufficient comparison data'
    assert np.all(np.diff(c('cycle'))==1),'Missing cycles in comparison'
    pitch=np.rad2deg(c('pitch'));v=c('speed')
    return dict(samples=len(a),pitch_mean_deg=float(np.mean(pitch)),pitch_std_deg=float(np.std(pitch)),
        pitch_peak_to_peak_deg=float(np.ptp(pitch)),speed_rms_m_s=float(np.sqrt(np.mean(v*v))),
        distance_range_m=float(np.ptp(c('distance'))),saturation_fraction=float(np.mean(c('saturated')!=0)),
        command_rms_A=np.sqrt(np.mean(np.stack([c('command_a'),c('command_b')],axis=1)**2,axis=0)).tolist(),
        integral_at_limit_fraction=float(np.mean(abs(c('velocity_integral'))>=1.199)),
        score=float(np.std(pitch)+20*np.sqrt(np.mean(v*v))))

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--velocity-followup',action='store_true')
    args=parser.parse_args()
    folder=HERE/'sessions'/('pid_compare_'+datetime.now().strftime('%Y%m%d_%H%M%S'));folder.mkdir()
    link=Link(folder);base=None;current=None;stages=[];selected=None
    def save():
        (folder/'comparison.json').write_text(json.dumps(dict(stages=stages,baseline=base,selected=selected,
            utc=datetime.now(timezone.utc).isoformat(),scope='Observational online comparison; uninstrumented contact and slow drift can confound results.'),indent=2))
    def apply(g):
        nonlocal current
        current=g.copy()
        line='GAINSSET '+' '.join(format(g[n],'.7g') for n in NAMES)
        assert len(line.encode('ascii'))<95,'Command exceeds firmware UART buffer'
        ack=link.command(line)
        # Verify the full gains actually used in CRC-checked control telemetry.
        revision=int(re.search(r'revision=(\d+)',ack)[1]);deadline=time.monotonic()+1
        while time.monotonic()<deadline:
            link.pump(.03)
            if not link.observations:continue
            r=dict(zip(FIELDS2,link.observations[-1]))
            if r['revision']!=revision:continue
            actual={n:r[n] for n in NAMES}
            assert all(math.isclose(actual[n],g[n],rel_tol=2e-6,abs_tol=1e-7) for n in NAMES),actual
            current=actual;return
        raise TimeoutError('No telemetry confirming gain revision')
    def measure(label,g,duration=40):
        apply(g);first=link.last_cycle;started=time.monotonic();last_print=started
        while time.monotonic()-started<duration:
            link.pump(.05)
            if not link.observations:continue
            r=dict(zip(FIELDS2,link.observations[-1]))
            if r['fault']!=0 or r['mode']!=2 or int(r['flags'])&7!=7:
                raise RuntimeError('Controller no longer active; never automatically restart')
            if abs(r['pitch'])>math.radians(10) or abs(r['speed'])>.25:
                apply(base);raise RuntimeError('Conservative trial envelope exceeded; baseline gains restored')
            if time.monotonic()-last_print>10:
                print('TRIAL '+label+' '+str(round(time.monotonic()-started))+'s',flush=True);last_print=time.monotonic()
        rows=[v for v in link.observations if v[FIELDS2.index('cycle')]>first+1600]
        report=dict(label=label,gains=current,**metrics(rows));stages.append(report);save()
        print('TRIAL_RESULT '+json.dumps(report),flush=True)
        return report
    try:
        # Existing binary stream can start halfway through a frame on attach.
        link.pump(.3)
        for attempt in range(3):
            try:status=link.command('STATUS');break
            except TimeoutError:
                if attempt==2:raise
        assert all(s in status for s in ['calibrated=1','running=1','ready=1','fault=0']),status
        link.command('GAINS');link.pump(.05)
        assert link.observations,'No CRC-checked observations'
        observed=dict(zip(FIELDS2,link.observations[-1]))
        base={n:observed[n] for n in NAMES};current=base.copy()
        # Restore the designated baseline for the primary trial. Longer float
        # spellings can overrun the existing 96-byte firmware command buffer.
        original=json.loads((HERE/'profiles/uart_live_current_2A.json').read_text())['gains']
        assert any(math.isclose(base['pitch_kd'],v,abs_tol=.001) for v in [1.8,2.0]) and any(math.isclose(base['position_kp'],v,abs_tol=.001) for v in [.6,.45]),'Unexpected live baseline'
        if not args.velocity_followup:
            for n in ['pitch_trim_rad','pitch_kd','position_kp']:base[n]=original[n]
        (folder/'metadata.json').write_text(json.dumps(dict(status_before=status,gains_before=base,
            controller_period_s=.005,transport='BCU2 UART CRC32',current_limit_A=2,attached_without_restart=True),indent=2))
        link.command('UARTSTREAM 200');print('PID_COMPARE_SESSION '+str(folder),flush=True)
        # Require agreement with both bracketing baseline windows, no added saturation/current burden.
        def better(candidate,references):
            return all(candidate['score']<.9*r['score'] and candidate['speed_rms_m_s']<=1.1*r['speed_rms_m_s']+.001
                and max(candidate['command_rms_A'])<=1.25*max(r['command_rms_A'])+.02 for r in references) and candidate['saturation_fraction']==0
        if args.velocity_followup:
            a=measure('velocity_reference_before',base,30)
            g=base.copy();g['velocity_kp']=.4
            b=measure('velocity_P_0p4',g,40)
            a2=measure('velocity_reference_after',base,30)
            chosen=b if better(b,[a,a2]) else a2
        else:
            a=measure('baseline_A1',base)
            g=base.copy();g['pitch_kd']=2.0
            b=measure('D_2p0',g)
            a2=measure('baseline_A2',base)
            chosen=b if better(b,[a,a2]) else a2
            g=chosen['gains'].copy();g['position_kp']=.45
            c=measure('position_P_0p45',g)
            reference=measure('reference_return',chosen['gains'])
            if better(c,[chosen,reference]):chosen=c
        selected=chosen;apply(chosen['gains']);save()
        profile=dict(schema='current_pid_v1',period_s=.005,current_limit_A=2,gains=chosen['gains'],
            origin='Conservative live comparison; observational, disturbance recovery not quantified',validation=dict(session=str(folder),label=chosen['label']))
        (folder/'selected_profile.json').write_text(json.dumps(profile,indent=2))
        print('SELECTED '+json.dumps(selected),flush=True)
        print('COMPARISON_COMPLETE logging continues; no stop command',flush=True)
        due=time.monotonic()
        while True:
            link.pump(.05)
            if time.monotonic()>=due:link.command('STATUS');due=time.monotonic()+10
    except Exception:
        if base is not None and current!=base:
            try:apply(base)
            except Exception:pass
        save();raise
    finally:link.close()

if __name__=='__main__':main()
