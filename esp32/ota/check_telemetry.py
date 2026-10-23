"""Post-OTA 20 s BCW2/200 Hz regression with stopped motors."""
import json
import time
from client import HERE, DebugClient, stopped
from wifi_debug.client import FIELDS2


def run(host='192.168.114.243', seconds=20):
    before=stopped(host)
    c=DebugClient(host)
    try:
        c.command(f'STREAM2 UDP 200 {c.udp.getsockname()[1]}','STREAM')
        began=time.perf_counter()
        while time.perf_counter()-began<seconds: c.pump(.02)
        c.command('STREAM2 UDP 0','STREAM')
        after=c.command('STATUS')
        rows=[dict(zip(FIELDS2,v)) for received,v in c.records if began+1<=received<=began+seconds]
        assert len(rows)>0
        unique={r['sequence'] for r in rows};span=max(unique)-min(unique)+1
        faults=sum(r['fault']!=0 for r in rows)
        active=sum(r['mode']!=0 or bool(r['flags']&6) for r in rows)
        report=dict(seconds=seconds,requested_hz=200,frame_bytes=220,received=len(rows),
            unique_sequences=len(unique),lost_sequences=span-len(unique),
            received_hz=(len(rows)-1)/((rows[-1]['sent_us']-rows[0]['sent_us'])/1e6),
            control_hz=(rows[-1]['cycle']-rows[0]['cycle'])/((rows[-1]['sample_us']-rows[0]['sample_us'])/1e6),
            nonzero_fault_samples=faults,nonzero_active_samples=active,
            max_dt_us=max(r['dt_us'] for r in rows),max_imu_age_us=max(r['imu_age_us'] for r in rows),
            status_before=before,status_after=after)
        report['passed']=report['received_hz']>=190 and faults==0 and active==0
        (HERE/'results/post_ota_telemetry.json').write_text(json.dumps(report,indent=2))
        assert report['passed'],report
        print(json.dumps(report,indent=2))
    finally: c.close()


if __name__=='__main__': run()
