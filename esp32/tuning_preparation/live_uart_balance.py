"""Explicit serial calibration/start, then continuous logging and online commands."""
import argparse,json,queue,re,runpy,sys,threading,time
from datetime import datetime
from pathlib import Path
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'wifi_debug/.tools'))
import serial
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--flash',action='store_true')
parser.add_argument('--attach',action='store_true',help='Attach to the running car without reset, CAL, gain changes or BALANCE')
parser.add_argument('--auto-dump',type=float,default=0,help='Rolling capture interval in seconds; never stops control')
args=parser.parse_args()
if args.flash:runpy.run_path(str(HERE/'flash_uart_only.py'),run_name='__main__')

folder=HERE/'sessions'/('uart_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
folder.mkdir(parents=True)
s=serial.Serial(port=None,baudrate=115200,timeout=.02)
s.dtr=False;s.rts=False;s.port='COM10';s.open()
log=(folder/'uart.log').open('w',encoding='utf-8',buffering=1,newline='')
events=(folder/'events.jsonl').open('w',encoding='utf-8',buffering=1)
display_buffer='';rx_tail='';completed_dumps=0
def display(text):
 global display_buffer
 display_buffer+=text
 while '\n' in display_buffer:
  line,display_buffer=display_buffer.split('\n',1);line=line.strip()
  if line.startswith(('STATUS ','GAINS ','DONE ','FAULT ','CSV_BEGIN','CSV_END','CAL ','RUN ','DEBUG ')):
   print(line,flush=True)
def receive(seconds):
 global rx_tail,completed_dumps
 end=time.monotonic()+seconds;text=''
 while time.monotonic()<end:
  b=s.read(max(1,s.in_waiting))
  if b:
   chunk=b.decode(errors='replace');log.write(chunk);text+=chunk
   rx_tail=(rx_tail+chunk)[-96:]
   if 'CSV_END' in rx_tail:
    completed_dumps+=1;rx_tail=rx_tail.split('CSV_END')[-1]
 return text
def command(line,seconds=.12):
 s.write((line+'\n').encode());out=receive(seconds)
 events.write(json.dumps(dict(utc=datetime.now().isoformat(),monotonic=time.monotonic(),command=line,reply=out))+'\n')
 print('> '+line,flush=True);display(out);return out
def status():
 out=command('STATUS');match=re.findall(r'STATUS [^\r\n]+',out)
 if not match:raise RuntimeError('No serial STATUS reply')
 return dict(re.findall(r'(\w+)=([^ ]+)',match[-1]))
try:
 deadline=time.monotonic()+10
 while True:
  try:st=status();break
  except RuntimeError:
   if time.monotonic()>deadline:raise
   receive(.15)
 assert st['fault']=='0',st
 if not args.attach:
  assert st['running']=='0',st
  p=json.loads((HERE/'profiles/physical_model_candidate_2A.json').read_text())
  names=['velocity_kp','velocity_ki','pitch_kd','pitch_kp','pitch_ki','yaw_kp','yaw_ki','yaw_kd','position_kp','pitch_trim_rad']
  command('GAINSSET '+' '.join(format(p['gains'][n],'.7g') for n in names))
 if not args.attach and st['calibrated']!='1':
  deadline=time.monotonic()+8
  while st.get('imu_us')=='0':
   if time.monotonic()>deadline:raise TimeoutError('IMU startup not ready')
   st=status()
  ack=command('CAL');assert 'CAL accepted=1' in ack,ack
  deadline=time.monotonic()+12
  while time.monotonic()<deadline:
   st=status()
   if st['fault']!='0':raise RuntimeError(command('FAULT'))
   if st['calibrated']=='1' and st['running']=='0':break
  else:raise TimeoutError('Normal calibration did not complete')
 if not args.attach:command('BALANCE 0 2.0')
 gains=command('GAINS')
 (folder/'metadata.json').write_text(json.dumps(dict(port='COM10',baud=115200,wireless=False,
  control_period_s=.005,rolling_record_period_s=.025,attached_without_restart=args.attach,
  start_status=st,gains=gains,auto_dump_interval_s=args.auto_dump),indent=2))
 print('LIVE_UART_LOG '+str(folder),flush=True)
 commands=queue.Queue()
 def inputs():
  for line in sys.stdin:commands.put(line.strip())
 threading.Thread(target=inputs,daemon=True).start()
 due=time.monotonic()+1
 next_dump=time.monotonic();dump_pending=False;seen_end=completed_dumps
 while True:
  out=receive(.03)
  if out:display(out)
  if dump_pending and completed_dumps>seen_end:dump_pending=False;seen_end=completed_dumps
  while not commands.empty():
   line=commands.get()
   if line=='QUIT':raise SystemExit
   if line:command(line,.05)
  if not dump_pending and time.monotonic()>=due:
   command('STATUS');due=time.monotonic()+5
  if args.auto_dump>0 and not dump_pending and time.monotonic()>=next_dump:
   command('DUMP',.03);dump_pending=True;seen_end=completed_dumps
   next_dump=time.monotonic()+args.auto_dump
finally:
 # Closing the logger never sends STOP to an autonomous running controller.
 s.close();log.close();events.close()
