"""Persistent Wi-Fi tuning/logging. Never calibrates, starts motors, or STOPs on exit."""
import argparse,csv,json,queue,sys,threading,time
from datetime import datetime,timezone
from pathlib import Path
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'wifi_debug'))
from client import Client,FIELDS2,WIRE2

def profile_command(path):
    p=json.loads(Path(path).read_text(encoding='utf-8'))
    if p.get('schema')!='current_pid_v1':raise ValueError('Profile must declare current_pid_v1')
    command='GAINSSET '+' '.join(format(float(p['gains'][name]),'.7g') for name in FIELDS2[-10:])
    if len(command)>=96:raise ValueError('Profile command exceeds firmware line capacity')
    return command

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('host');p.add_argument('--seconds',type=float,default=60)
    p.add_argument('--interactive',action='store_true');p.add_argument('--apply',type=Path)
    p.add_argument('--usb',choices=['yes','no','unknown'],default='unknown')
    p.add_argument('--output',type=Path,default=HERE/'sessions'/datetime.now().strftime('%Y%m%d_%H%M%S'))
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    c=Client(args.host);commands=queue.Queue();start=time.perf_counter();rows=0
    meta=dict(schema='current_pid_v1',telemetry='BCW2',utc=datetime.now(timezone.utc).isoformat(),
              host=args.host,usb_connected=args.usb,controller_period_s=.005,requested_hz=200,
              notes='Controller remains autonomous. No automatic CAL/BALANCE/STOP.')
    try:
        meta['status_before']=c.command('STATUS')
        meta['gains_before']=c.command('GAINS')
        if 'schema=current_pid_v1' not in meta['gains_before']:raise RuntimeError('Wrong gain units/firmware')
        (args.output/'metadata.json').write_text(json.dumps(meta,indent=2),encoding='utf-8')
        if args.apply:commands.put(profile_command(args.apply))
        c.command(f'STREAM2 UDP 200 {c.udp.getsockname()[1]}','STREAM')
        def input_loop():
            for line in sys.stdin:
                commands.put(line.strip())
                if line.strip()=='QUIT':break
        if args.interactive:threading.Thread(target=input_loop,daemon=True).start()
        with (args.output/'samples.csv').open('w',newline='',encoding='utf-8') as sample_file, \
             (args.output/'events.jsonl').open('w',encoding='utf-8') as events:
            writer=csv.writer(sample_file);writer.writerow(['received_monotonic',*FIELDS2])
            end=start+args.seconds;quit_requested=False
            while time.perf_counter()<end and not quit_requested:
                while not commands.empty():
                    text=commands.get()
                    if text=='QUIT':quit_requested=True;break
                    if not text:continue
                    if text.startswith('MARK '):
                        events.write(json.dumps(dict(monotonic=time.perf_counter(),marker=text[5:]))+'\n');events.flush();continue
                    if text.startswith('APPLY '):text=profile_command(text[6:])
                    event=dict(monotonic=time.perf_counter(),command=text)
                    try:event['reply']=c.command(text)
                    except (RuntimeError,TimeoutError) as e:event['error']=str(e)
                    print(json.dumps(event),flush=True);events.write(json.dumps(event)+'\n');events.flush()
                c.pump(.02)
                for received,values in c.records:
                    if len(values)!=len(FIELDS2):raise ValueError('Unexpected telemetry format')
                    writer.writerow([received,*values]);rows+=1
                c.records.clear();c.lines.clear();sample_file.flush()
            meta['net_after']=c.command('NET')
            c.command('STREAM2 UDP 0','STREAM')
            meta['status_after']=c.command('STATUS');meta['rows']=rows
    finally:
        # Closing the subscription leaves autonomous balance running, if active.
        c.close();meta['closed_utc']=datetime.now(timezone.utc).isoformat()
        (args.output/'metadata.json').write_text(json.dumps(meta,indent=2),encoding='utf-8')
    print(json.dumps(dict(folder=str(args.output),rows=rows),indent=2))

if __name__=='__main__':main()
