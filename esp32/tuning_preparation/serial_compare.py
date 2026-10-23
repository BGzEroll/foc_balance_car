"""Bounded, wheels-free calibration comparison; captures actual UART errors."""
import argparse,json,sys,time,re
from pathlib import Path
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'wifi_debug/.tools'))
import serial
p=argparse.ArgumentParser();p.add_argument('--wifi-off',action='store_true');p.add_argument('--monitor-off',action='store_true');p.add_argument('--output',type=Path,required=True)
a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
s=serial.Serial(port=None,baudrate=115200,timeout=.02);s.dtr=s.rts=False;s.port='COM10';s.open();logs=[];events=[]
def read_for(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        b=s.read(max(s.in_waiting,1))
        if b: logs.append(b.decode(errors='replace'))
def command(text,seconds=.3):
    before=len(logs);s.write((text+'\n').encode());read_for(seconds)
    out=''.join(logs[before:]);events.append(dict(command=text,reply=out));print(out,flush=True);return out
try:
    s.rts=True;time.sleep(.15);s.rts=False;read_for(7)
    if a.wifi_off:command('WIFIOFF')
    if a.monitor_off:command('MONITOROFF')
    out=command('STATUS');assert 'running=0' in out and 'fault=0' in out
    command('CAL');end=time.monotonic()+12
    while time.monotonic()<end:
        out=command('STATUS',.5)
        if re.search(r'fault=[1-9]\d*',out) or 'calibrated=1 running=0' in out:break
    command('STOP');command('FAULT');command('STATUS')
finally:
    s.write(b'STOP\n');read_for(.2);s.close()
    (a.output/'uart.log').write_text(''.join(logs),encoding='utf-8')
    (a.output/'result.json').write_text(json.dumps(dict(wifi_off=a.wifi_off,monitor_off=a.monitor_off,events=events),indent=2))
