"""Capture encoder bursts through the existing UART logger without reopening the port."""
import json,os,shutil,sys,time
from pathlib import Path

source=Path(sys.argv[1]);label=sys.argv[2];h=Path(__file__).resolve().parent
out=h/'results/encoder_shadow'/label;out.mkdir(parents=True,exist_ok=True)
log=(source/'commands.log').open()
def command(text):
 log.seek(0,2);fd=os.open('/dev/ttyACM0',os.O_RDWR|os.O_NOCTTY|os.O_NONBLOCK)
 try:os.write(fd,(text+'\n').encode())
 finally:os.close(fd)
 pending='';deadline=time.monotonic()+5
 while time.monotonic()<deadline:
  pending+=log.read()
  for line in pending.splitlines():
   if line.startswith(text.split()[0]+' '):return line
  time.sleep(.03)
 raise TimeoutError(text)
for _ in range(6):
 before=set(source.glob('encoder_*.bin'))
 assert 'accepted=1' in command('ENCCAP')
 deadline=time.monotonic()+5
 while time.monotonic()<deadline:
  time.sleep(.25);reply=command('ENCDUMP')
  if 'ready=1' in reply:break
 else:raise TimeoutError('Encoder capture did not complete')
 files=set(source.glob('encoder_*.bin'))-before;assert len(files)==1,files
 p=files.pop();shutil.copy2(p,out/p.name);print('CAPTURED '+str(out/p.name),flush=True)
 time.sleep(.2)
print('ENCODER_BURSTS_COMPLETE '+str(out),flush=True)
