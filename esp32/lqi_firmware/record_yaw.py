"""Attach to the running car without changing DTR/RTS; record yaw and encoders."""
import argparse,csv,json,os,pathlib,queue,select,signal,struct,sys,termios,threading,time,zlib
from telemetry import FIELDS2,WIRE2,YAW_WIRE,YAW_FIELDS,ATTITUDE_WIRE,ATTITUDE_FIELDS,IMU_WIRE,IMU_FIELDS,TORQUE_WIRE,TORQUE_FIELDS
from live_feed import forward

parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--overnight',action='store_true',help='Detached long run: save about 20Hz CSV, no full raw binary, no motor commands');args=parser.parse_args()
h=pathlib.Path(__file__).resolve().parent
folder=h/'results'/(('overnight_yaw_' if args.overnight else 'yaw_')+time.strftime('%Y%m%d_%H%M%S'));folder.mkdir()
fd=os.open('/dev/ttyACM0',os.O_RDWR|os.O_NOCTTY)
settings=termios.tcgetattr(fd);settings[0]=settings[1]=settings[3]=0
settings[2]=(settings[2]&~(termios.PARENB|termios.CSTOPB|termios.CSIZE|termios.HUPCL))|termios.CS8|termios.CREAD|termios.CLOCAL
settings[4]=settings[5]=termios.B921600;settings[6][termios.VMIN]=settings[6][termios.VTIME]=0
termios.tcsetattr(fd,termios.TCSANOW,settings)
ywire=YAW_WIRE;yfields=YAW_FIELDS
yc=(folder/'yaw.csv').open('w',newline='');yw=csv.writer(yc);yw.writerow(yfields)
awire=ATTITUDE_WIRE;afields=ATTITUDE_FIELDS
raw=None if args.overnight else (folder/'serial.bin').open('wb');log=(folder/'commands.log').open('w',buffering=1)
bc=(folder/'samples.csv').open('w',newline='');ac=(folder/'attitude.csv').open('w',newline='')
bw=csv.writer(bc);aw=csv.writer(ac);bw.writerow(FIELDS2);aw.writerow(afields)
ic=(folder/'imu.csv').open('w',newline='');iw=csv.writer(ic);iw.writerow(IMU_FIELDS)
tc=(folder/'torque.csv').open('w',newline='');tw=csv.writer(tc);tw.writerow(TORQUE_FIELDS)
commands=queue.Queue();buf=bytearray();frames=0;bad=0;last=None;quit_requested=False;reported_fault=0
last_bc_write=last_yaw_write=0;next_status=time.monotonic()+15;next_attitude=time.monotonic()+60
def save():
 (folder/'metadata.json').write_text(json.dumps(dict(pid=os.getpid(),overnight=args.overnight,frames=frames,crc_errors=bad,last=last),indent=2))
def stop(signum,frame):
 global quit_requested
 quit_requested=True
signal.signal(signal.SIGTERM,stop)
def inputs():
 for line in sys.stdin:commands.put(line.strip())
threading.Thread(target=inputs,daemon=True).start()
def send(text):os.write(fd,(text+'\n').encode())
send('STATUS');send('UARTSTREAM 200');send('ATTSTREAM 0');send('YAWSTREAM 50')
print('YAW_CAPTURE '+str(folder),flush=True)
try:
 while not quit_requested:
  if select.select([fd],[],[],.03)[0]:
   data=os.read(fd,65536)
   if raw:raw.write(data)
   buf.extend(data)
  while buf:
   if buf.startswith(b'TOR1'):
    size=TORQUE_WIRE.size
    if len(buf)<size:break
    packet=bytes(buf[:size]);del buf[:size]
    if zlib.crc32(packet[:-4])&0xffffffff!=struct.unpack_from('<I',packet,size-4)[0]:bad+=1;continue
    forward(packet);tw.writerow(TORQUE_WIRE.unpack(packet));continue
   if buf.startswith(b'IMU2'):
    size=IMU_WIRE.size
    if len(buf)<size:break
    packet=bytes(buf[:size]);del buf[:size]
    if zlib.crc32(packet[:-4])&0xffffffff!=struct.unpack_from('<I',packet,size-4)[0]:bad+=1;continue
    forward(packet);iw.writerow(IMU_WIRE.unpack(packet));continue
   size=224 if buf.startswith(b'BCU3') else 124 if buf.startswith(b'ATT2') else 68 if buf.startswith(b'YAW3') else 0
   if size:
    if len(buf)<size:break
    packet=bytes(buf[:size]);del buf[:size]
    if zlib.crc32(packet[:-4])&0xffffffff!=struct.unpack_from('<I',packet,size-4)[0]:bad+=1;continue
    forward(packet)
    if size==224:
     values=WIRE2.unpack(packet[:-4]);last=dict(zip(FIELDS2,values));frames+=1
     if not args.overnight or last['sample_us']-last_bc_write>=50000:
      bw.writerow(values);last_bc_write=last['sample_us']
    elif size==124:aw.writerow(awire.unpack(packet))
    else:
     values=ywire.unpack(packet)
     if not args.overnight or values[2]-last_yaw_write>=50000:
      yw.writerow(values);last_yaw_write=values[2]
    continue
   positions=[p for p in [buf.find(b'BCU3'),buf.find(b'ATT2'),buf.find(b'YAW3'),buf.find(b'IMU2'),buf.find(b'TOR1')] if p>=0]
   magic=min(positions) if positions else -1;nl=buf.find(b'\n')
   if nl>=0 and (magic<0 or nl<magic):
    line=bytes(buf[:nl]).decode(errors='replace').strip();del buf[:nl+1];log.write(line+'\n');print(line,flush=True)
   elif magic>0:del buf[:magic]
   elif len(buf)>4096:del buf[:-3]
   else:break
  if last and last['fault'] and last['fault']!=reported_fault:
   reported_fault=last['fault'];send('FAULT');save()
  bc.flush();ac.flush();yc.flush();ic.flush();tc.flush()
  if raw:raw.flush()
  if args.overnight and time.monotonic()>=next_status:
   send('STATUS');save();next_status=time.monotonic()+15
  if args.overnight and time.monotonic()>=next_attitude:
   send('ATTITUDE');send('YAW');next_attitude=time.monotonic()+60
  while not commands.empty():
   text=commands.get()
   if text=='QUIT':quit_requested=True
   elif text:send(text)
finally:
 send('ATTSTREAM 0');os.close(fd)
 save()
 if raw:raw.close()
 log.close();bc.close();ac.close();yc.close();ic.close();tc.close()
