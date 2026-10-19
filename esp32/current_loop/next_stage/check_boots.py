"""No motor commands: repeated serial opens and one stopped-state I2C NACK."""
from pathlib import Path
import argparse,serial,time,json
p=argparse.ArgumentParser();p.add_argument('label');p.add_argument('--n',type=int,default=8)
p.add_argument('--nack',action='store_true');a=p.parse_args();root=Path(__file__).resolve().parent
results=[];logs=bytearray()
for k in range(a.n):
 s=serial.Serial();s.port='/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A33014245-if00'
 s.baudrate=115200;s.timeout=.05;s.dtr=False;s.rts=False;s.open();data=bytearray()
 def read_for(sec):
  end=time.monotonic()+sec
  while time.monotonic()<end:data.extend(s.read(65536))
 def send(x):s.write((x+'\n').encode());s.flush()
 def parse(prefix):return [json.loads(x[len(prefix):]) for x in data.decode(errors='replace').splitlines() if x.startswith(prefix)]
 read_for(2);send('STOP');read_for(.15)
 before=len(parse('HEALTH '))
 for q in range(4):send('STATUS');read_for(.12)
 status=parse('STATUS ')[-4:];health=parse('HEALTH ')[before:]
 assert status and all(z['running']==0 and z['requested']==0 for z in status)
 injection=None
 if a.nack:
  assert health,'Health telemetry missing'
  h0=health[-1]
  old=len(data);send('SENSOR_NACK');read_for(.15)
  assert b'SENSOR_NACK_QUEUED' in data[old:]
  send('STATUS');read_for(.15);send('STATUS');read_for(.15)
  status=parse('STATUS ')[-2:];health=parse('HEALTH ')[-2:]
  assert health[-1]['left_recoveries']>h0['left_recoveries'],'NACK did not exercise recovery'
  assert health[-1]['left_updates']>h0['left_updates'],'No fresh encoder updates after NACK'
  injection={'before':h0,'after':health[-1]}
 send('STOP');read_for(.1);s.close();logs.extend(data)
 good=bool(status) and all(z['left_age_us']<1500 and z['bus_V']>5 for z in status)
 if health:good=good and all(z['right_age_us']<2000 for z in health)
 results.append({'boot':k,'fresh':good,'status':status,'health':health,'injection':injection})
 print(json.dumps(results[-1]),flush=True)
(root/f'{a.label}.log').write_bytes(logs)
(root/f'{a.label}.json').write_text(json.dumps(results,indent=2))
print('RESULT',sum(z['fresh'] for z in results),'/',a.n)
