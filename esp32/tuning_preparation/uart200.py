"""921600-baud CRC-checked 200Hz logging, live commands, no STOP on logger exit."""
import argparse,csv,json,queue,runpy,struct,sys,threading,time,zlib
from collections import deque
from datetime import datetime,timezone
from pathlib import Path
HERE=Path(__file__).resolve().parent
sys.path[:0]=[str(HERE.parent/'wifi_debug/.tools'),str(HERE.parent/'wifi_debug')]
import serial
from client import FIELDS2,WIRE2

class Link:
 def __init__(self,folder):
  self.s=serial.Serial(port=None,baudrate=921600,timeout=.01)
  self.s.dtr=self.s.rts=False;self.s.port='COM10';self.s.open()
  self.raw=(folder/'serial.bin').open('wb');self.ascii=(folder/'commands.log').open('w',encoding='utf-8')
  self.csv=(folder/'samples.csv').open('w',newline='',encoding='utf-8');self.writer=csv.writer(self.csv)
  self.writer.writerow(['received_monotonic',*FIELDS2]);self.events=(folder/'events.jsonl').open('w',encoding='utf-8')
  self.buffer=bytearray();self.lines=[];self.rows=0;self.bad_crc=0;self.dropped=0;self.last_cycle=None;self.times=[]
  self.observations=deque(maxlen=16000)
 def pump(self,seconds=.02):
  end=time.monotonic()+seconds
  while time.monotonic()<end:
   data=self.s.read(max(1,self.s.in_waiting))
   if data:self.raw.write(data);self.buffer.extend(data)
   while self.buffer:
    if self.buffer.startswith(b'BCU2'):
     if len(self.buffer)<224:break
     data=self.buffer[:220];crc=struct.unpack_from('<I',self.buffer,220)[0]
     if zlib.crc32(data)&0xffffffff!=crc:self.bad_crc+=1;del self.buffer[0];continue
     values=WIRE2.unpack(data);del self.buffer[:224];now=time.perf_counter()
     self.writer.writerow([now,*values]);self.rows+=1;self.times.append(now)
     self.observations.append(values)
     cycle=values[FIELDS2.index('cycle')]
     if self.last_cycle is not None and cycle>self.last_cycle:self.dropped+=max(0,cycle-self.last_cycle-1)
     self.last_cycle=cycle
    else:
     magic=self.buffer.find(b'BCU2');nl=self.buffer.find(b'\n')
     if nl>=0 and (magic<0 or nl<magic):
      line=self.buffer[:nl].decode('ascii',errors='replace').strip();del self.buffer[:nl+1]
      self.lines.append(line);self.ascii.write(line+'\n')
      if line.startswith(('STATUS ','GAINS ','FAULT ','RUN ','CAL ','UARTSTREAM ','DONE ')):print(line,flush=True)
     elif magic>0:del self.buffer[:magic]
     elif len(self.buffer)>8192:del self.buffer[:-3]
     else:break
  self.csv.flush();self.raw.flush();self.ascii.flush()
 def command(self,text,prefix=None,timeout=3):
  if len(text.encode('ascii'))>=95:raise ValueError('Command exceeds 96-byte firmware UART line buffer')
  before=len(self.lines);self.s.write((text+'\n').encode());start=time.monotonic()
  prefix=prefix or {'GAINSSET':'GAINS','SET':'GAINS','BALANCE':'RUN'}.get(text.split()[0],text.split()[0])
  while time.monotonic()-start<timeout:
   self.pump(.02)
   for line in self.lines[before:]:
    if line.startswith(('REJECT','BUSY')):raise RuntimeError(line)
    if line.startswith(prefix):
     self.events.write(json.dumps(dict(utc=datetime.now(timezone.utc).isoformat(),monotonic=time.perf_counter(),command=text,reply=line))+'\n');self.events.flush();return line
  raise TimeoutError(text)
 def close(self):
  self.s.close();self.csv.close();self.raw.close();self.ascii.close();self.events.close()

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--flash',action='store_true');p.add_argument('--attach',action='store_true')
 p.add_argument('--profile',type=Path,default=HERE/'profiles/uart_live_current_2A.json');a=p.parse_args()
 if a.flash:runpy.run_path(str(HERE/'flash_uart_only.py'),run_name='__main__')
 folder=HERE/'sessions'/('uart200_'+datetime.now().strftime('%Y%m%d_%H%M%S'));folder.mkdir(parents=True)
 c=Link(folder);meta=dict(transport='BCU2 UART CRC32',baud=921600,usb_connected='yes',controller_period_s=.005,requested_hz=200,tilt_limit_deg=45,wheel_limit_rad_s=40,phase_trip_A=2.5,current_limit_A=2)
 try:
  end=time.monotonic()+12
  while True:
   try:st=c.command('STATUS');break
   except (TimeoutError,RuntimeError):
    if time.monotonic()>end:raise
  meta['status_before']=st;meta['gains_before']=c.command('GAINS')
  c.command('UARTSTREAM 200')
  if not a.attach:
   assert 'fault=0' in st and 'running=0' in st,st
   profile=json.loads(a.profile.read_text());names=FIELDS2[-10:]
   c.command('GAINSSET '+' '.join(format(profile['gains'][k],'.7g') for k in names))
   end=time.monotonic()+8
   while 'imu_us=0 ' in st:
    if time.monotonic()>end:raise TimeoutError('IMU initialization')
    c.pump(.1);st=c.command('STATUS')
   if 'calibrated=1' not in st:
    assert 'accepted=1' in c.command('CAL');end=time.monotonic()+12
    while True:
     c.pump(.12);st=c.command('STATUS')
     if 'fault=0' not in st:raise RuntimeError(c.command('FAULT'))
     if 'calibrated=1' in st and 'running=0' in st:break
     if time.monotonic()>end:raise TimeoutError('Calibration')
   c.command('BALANCE 0 2.0');meta['applied_profile']=profile
  (folder/'metadata.json').write_text(json.dumps(meta,indent=2));print('UART200_SESSION '+str(folder),flush=True)
  commands=queue.Queue()
  def inputs():
   for line in sys.stdin:commands.put(line.strip())
  threading.Thread(target=inputs,daemon=True).start();due=time.monotonic()+1
  while True:
   c.pump(.05)
   while not commands.empty():
    text=commands.get()
    if text=='QUIT':return
    if text:c.command(text)
   if time.monotonic()>=due:
    c.command('STATUS');recent=[v for v in c.times if v>time.perf_counter()-10]
    hz=(len(recent)-1)/(recent[-1]-recent[0]) if len(recent)>1 else 0
    print(json.dumps(dict(frames=c.rows,recent_hz=round(hz,2),crc_errors=c.bad_crc,missing_cycles=c.dropped)),flush=True)
    c.times=recent;due=time.monotonic()+10
 finally:
  meta.update(rows=c.rows,crc_errors=c.bad_crc,missing_cycles=c.dropped,closed_utc=datetime.now(timezone.utc).isoformat())
  (folder/'metadata.json').write_text(json.dumps(meta,indent=2));c.close()

if __name__=='__main__':main()
