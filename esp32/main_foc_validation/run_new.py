"""Run one bounded new-core test, retaining startup, calibration, metrics and CSV."""
from pathlib import Path
import argparse,serial,time,json
p=argparse.ArgumentParser();p.add_argument('label');p.add_argument('--command',required=True);p.add_argument('--wide',action='store_true');a=p.parse_args()
root=Path(__file__).resolve().parent
s=serial.Serial();s.port='/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A33014245-if00';s.baudrate=115200;s.timeout=.04;s.dtr=False;s.rts=False
data=bytearray();s.open()
def read_for(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:data.extend(s.read(65536))
def send(x):s.write((x+'\n').encode());s.flush()
def parse(key):
 return [json.loads(l[len(key)+1:]) for l in data.decode(errors='replace').splitlines() if l.startswith(key+' ')]
try:
 read_for(6);send('STOP');read_for(.2);send('STATUS');read_for(.2)
 st=parse('STATUS')[-1];assert not st['fault'] and not st['running'] and not st['requested'],st
 send('ARM');end=time.monotonic()+9
 while time.monotonic()<end:
  read_for(.1)
  if b'ALIGN_FAILED' in data:raise RuntimeError('Alignment failed')
  if b'ALIGNED\r\n' in data or b'ALIGNED\n' in data:break
 else:raise RuntimeError('Alignment timed out')
 read_for(.2);st=parse('STATUS')[-1];assert st['aligned']==[1,1] and not st['fault'],st
 if a.wide:
  send('WIDE');read_for(.2);assert b'WIDE_OK' in data
 before=len(data);sent_at=time.monotonic();send(a.command);read_for(.15)
 while b'ACCEPTED' not in data[before:] and time.monotonic()-sent_at<7:read_for(.05)
 ack_seconds=time.monotonic()-sent_at
 assert b'ACCEPTED' in data[before:],data[before:]
 end=time.monotonic()+8
 while time.monotonic()<end:
  read_for(.1)
  if b'DONE' in data[before:]:break
 else:raise RuntimeError('Run timeout')
 send('STOP');read_for(.2);send('DUMP');end=time.monotonic()+35
 while time.monotonic()<end:
  read_for(.1)
  if b'\nEND\r\n' in data or b'\nEND\n' in data:break
 else:raise RuntimeError('Dump incomplete')
 send('STOP');read_for(.2)
finally:
 send('STOP');read_for(.1);s.close();(root/(a.label+'.log')).write_bytes(data)
lines=data.decode(errors='replace').splitlines();header=next(l[4:] for l in lines if l.startswith('CSV '));rows=[l[2:] for l in lines if l.startswith('R ')]
(root/(a.label+'.csv')).write_text(header+'\n'+'\n'.join(rows)+'\n')
out={key:parse(key) for key in ['STATUS','METRIC','META']};out['command']=a.command;out['ack_seconds']=ack_seconds
(root/(a.label+'.json')).write_text(json.dumps(out,indent=2))
print(json.dumps({'label':a.label,'status':out['STATUS'][-1],'metric':out['METRIC'],'meta':out['META']}))
assert out['META'][-1]['rows']==len(rows) and len(rows)>0
assert not out['STATUS'][-1]['fault'] and not out['STATUS'][-1]['running'] and not out['STATUS'][-1]['requested']
assert not any(x in data for x in [b'watchdog got triggered',b'Guru Meditation',b'panic'])
