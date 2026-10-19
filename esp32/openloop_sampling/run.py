"""Explicit bounded serial command; DUMP only after output has stopped."""
from pathlib import Path
import argparse,json,time,serial,csv
p=argparse.ArgumentParser()
p.add_argument('label');p.add_argument('--v',type=float,default=0)
p.add_argument('--speed',type=float,default=0);p.add_argument('--ms',type=int,default=1000)
p.add_argument('--decimate',type=int,default=1);p.add_argument('--start',type=int,default=0)
p.add_argument('--peek',action='store_true');p.add_argument('--reset',action='store_true')
a=p.parse_args();root=Path(__file__).resolve().parent
port='/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A33014245-if00'
s=serial.Serial();s.port=port;s.baudrate=115200;s.timeout=.2;s.dtr=False;s.rts=False;s.open()
data=bytearray()
def read_for(seconds):
 until=time.monotonic()+seconds
 while time.monotonic()<until:data.extend(s.read(65536))
def send(x):s.write((x+'\n').encode());s.flush()
if a.reset:
 s.rts=True;time.sleep(.1);s.rts=False;read_for(4)
else:
 # CH343/USB-IP may pulse reset while opening the port. Wait for initialization
 # before sending commands; otherwise an early STATUS could be lost at boot.
 read_for(2)
send('STOP');read_for(.4);send('STATUS');read_for(.4)
initial=[]
for line in data.decode(errors='replace').splitlines():
 if line.startswith('STATUS '):initial.append(json.loads(line[7:]))
if not initial or initial[-1]['running'] or initial[-1]['requested']:
 (root/f'{a.label}.log').write_bytes(data);s.close()
 raise RuntimeError('No confirmed stopped STATUS; RUN was not sent')
if not a.peek:
 send(f'RUN {a.v} {a.speed} {a.ms} {a.decimate} {a.start}');read_for(a.ms/1000+.8)
 send('STOP');read_for(.4);send('DUMP')
 until=time.monotonic()+100
 while time.monotonic()<until:
  data.extend(s.read(65536))
  if b'\nEND\r\n' in data or b'\nEND\n' in data:break
 send('STATUS');read_for(.4)
s.close();text=data.decode(errors='replace');(root/f'{a.label}.log').write_text(text,encoding='utf-8')
meta=None;rows=[];luts={};statuses=[]
for line in text.splitlines():
 if line.startswith('META '):meta=json.loads(line[5:])
 elif line.startswith('LUT '):
  cells=line[4:].split(',');luts[cells[0]]=list(map(int,cells[1:]))
 elif line.startswith('R '):rows.append(line[2:])
 elif line.startswith('CSV '):header=line[4:]
 elif line.startswith('STATUS '):statuses.append(json.loads(line[7:]))
if meta:
 assert len(rows)==meta['rows'],(len(rows),meta)
 (root/f'{a.label}.csv').write_text(header+'\n'+'\n'.join(rows)+'\n')
 (root/f'{a.label}.json').write_text(json.dumps({'meta':meta,'lut_mv':luts,'status':statuses},indent=2))
print(json.dumps({'label':a.label,'meta':meta,'statuses':statuses},ensure_ascii=False))
bad=[x for x in ['Guru Meditation','assert failed','panic','abort()'] if x in text]
if bad or (not a.peek and (not meta or not rows)):
 print(text[-5000:]);raise SystemExit(1)
