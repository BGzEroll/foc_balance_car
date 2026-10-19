"""Bounded MA experiment; outputs remain stopped between invocations."""
from pathlib import Path
import argparse,json,time,serial
p=argparse.ArgumentParser()
p.add_argument('label');p.add_argument('--command');p.add_argument('--zero',type=float)
p.add_argument('--gains',nargs=4,type=float);p.add_argument('--reset',action='store_true')
a=p.parse_args();root=Path(__file__).resolve().parent
s=serial.Serial();s.port='/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A33014245-if00'
s.baudrate=115200;s.timeout=.2;s.dtr=False;s.rts=False;s.open();data=bytearray()
def read_for(seconds):
 until=time.monotonic()+seconds
 while time.monotonic()<until:data.extend(s.read(65536))
def send(x):s.write((x+'\n').encode());s.flush()
def statuses():
 return [json.loads(l[7:]) for l in data.decode(errors='replace').splitlines() if l.startswith('STATUS ')]
def setting(x,ack):
 before=len(data);send(x);read_for(.3)
 if ack.encode() not in data[before:]:raise RuntimeError('Setting not acknowledged: '+x)
try:
 if a.reset:s.rts=True;time.sleep(.1);s.rts=False
 read_for(3 if a.reset else 2)
 send('STOP');read_for(.3);send('STATUS');read_for(.3)
 st=statuses()
 if not st or st[-1]['running'] or st[-1]['requested']:raise RuntimeError('No confirmed stopped status')
 if a.zero is not None:setting(f'ZERO {a.zero:.9f}','ZERO_OK')
 if a.gains:setting('GAINS '+' '.join(map(str,a.gains)),'GAINS_OK')
 if a.command:
  before=len(data);send(a.command);duration=int(a.command.split()[2]);read_for(duration/1000+.5)
  if b'ACCEPTED' not in data[before:]:raise RuntimeError('Experiment not accepted')
  send('STOP');read_for(.3)
  if statuses()[-1]['running'] or statuses()[-1]['requested']:raise RuntimeError('Not stopped before dump')
  send('DUMP');until=time.monotonic()+100
  while time.monotonic()<until:
   data.extend(s.read(65536))
   if b'\nEND\r\n' in data or b'\nEND\n' in data:break
  send('STATUS');read_for(.3)
finally:
 send('STOP');read_for(.2);s.close()
 (root/f'{a.label}.log').write_bytes(data)
text=data.decode(errors='replace');meta=None;rows=[];luts={};header=None
for line in text.splitlines():
 if line.startswith('META '):meta=json.loads(line[5:])
 elif line.startswith('LUT '):
  x=line[4:].split(',');luts[x[0]]=list(map(int,x[1:]))
 elif line.startswith('CSV '):header=line[4:]
 elif line.startswith('R '):rows.append(line[2:])
if meta:
 assert len(rows)==meta['rows'] and len(rows)>0,(len(rows),meta)
 (root/f'{a.label}.csv').write_text(header+'\n'+'\n'.join(rows)+'\n')
 (root/f'{a.label}.json').write_text(json.dumps({'meta':meta,'lut_mv':luts,'status':statuses()},indent=2))
print(json.dumps({'label':a.label,'meta':meta,'status':statuses()},ensure_ascii=False))
if any(x in text for x in ['Guru Meditation','assert failed','panic','abort()']) or (a.command and not meta):
 print(text[-5000:]);raise SystemExit(1)
