"""One bounded calibration on an operator-confirmed wheels-free car; never starts balance."""
import argparse,csv,json,sys,time
from pathlib import Path
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'wifi_debug'))
from client import Client,FIELDS2

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--host',default='192.168.114.243')
p.add_argument('--stream',action='store_true')
p.add_argument('--output',type=Path,required=True)
args=p.parse_args();args.output.mkdir(parents=True,exist_ok=False)
c=Client(args.host);result={'telemetry_enabled':args.stream,'statuses':[]};started=False
try:
    s=c.command('STATUS');print(s,flush=True);result['before']=s
    assert 'running=0' in s and 'active=0' in s and 'fault=0' in s
    result['gains']=c.command('GAINS')
    if args.stream:c.command(f'STREAM2 UDP 200 {c.udp.getsockname()[1]}','STREAM')
    result['cal']=c.command('CAL');started=True;print(result['cal'],flush=True)
    end=time.monotonic()+12;due=0
    while time.monotonic()<end:
        c.pump(.02)
        if time.monotonic()<due:continue
        due=time.monotonic()+.5
        s=c.command('STATUS');result['statuses'].append(s);print(s,flush=True)
        if 'fault=0' not in s or ('calibrated=1' in s and 'running=0' in s):break
    result['stop']=c.command('STOP')
    result['fault']=c.command('FAULT');print(result['fault'],flush=True)
    result['after']=c.command('STATUS');print(result['after'],flush=True)
    result['passed']=all(k in result['after'] for k in ['calibrated=1','running=0','fault=0'])
finally:
    if started:
        try:c.command('STOP')
        except Exception as exc:result['final_stop_error']=str(exc)
    c.close()
    (args.output/'result.json').write_text(json.dumps(result,indent=2))
    if args.stream:
        with (args.output/'samples.csv').open('w',newline='') as f:
            w=csv.writer(f);w.writerow(['received_monotonic',*FIELDS2])
            w.writerows([t,*v] for t,v in c.records)
