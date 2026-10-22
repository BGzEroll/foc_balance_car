"""Run on develop inside perf_evidence after LUT tests have stopped."""
from pathlib import Path
import json,subprocess
p=Path(__file__).resolve().parent
root=next(x for x in p.parents if (x/'.git').exists())
assert subprocess.check_output(['git','branch','--show-current'],cwd=root,text=True).strip()=='codex/test-main-foc-perf-240-lut'
assert (p/'before_flash.bin').stat().st_size==0x200000
state=json.loads((p/'lut_speed10.json').read_text())['STATUS'][-1]
assert not state['running'] and not state['requested'] and not state['fault']
python='/home/bgzerol/.espressif/tools/python/v6.0.2/venv/bin/python3'
with (p/'reference_flash.log').open('w') as log:
    subprocess.run([python,'-m','esptool','--chip','esp32s3','--port','/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A33014245-if00','--baud','460800','write-flash','0x10000',str(p/'reference_v18.bin')],stdout=log,stderr=subprocess.STDOUT,check=True)
for label,speed in [('reference_speed10',10),('reference_speed27',27.5)]:
    args=[python,str(p/'run_reference.py'),label,'--reset','--mask','3',
        '--zero','1',str(-state['direction'][0]),str(state['zero'][0]),
        '--zero','2',str(state['direction'][1]),str(state['zero'][1]),
        '--limit','.04','.4','--velocity','.2','.4','.7','--command',f'SPEED {speed} 4000 25']
    with (p/(label+'_runner.log')).open('w') as log:
        subprocess.run(args,stdout=log,stderr=subprocess.STDOUT,check=True)
    d=json.loads((p/(label+'.json')).read_text())
    assert d['meta']['version']==18 and d['meta']['reason']==1
    assert not d['status'][-1]['running'] and not d['status'][-1]['requested']
    print(label,'PASS; full-rate valid',d['meta']['full_rate_valid'],flush=True)
