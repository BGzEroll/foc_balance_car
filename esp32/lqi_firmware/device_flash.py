"""Flash the already-stopped car with verified PID; never CAL/BALANCE/PROBE."""
import contextlib,hashlib,io,json,re,struct,subprocess,sys,time,zlib
from datetime import datetime,timezone
from pathlib import Path
H=Path(__file__).resolve().parent;R=H.parents[1];FW=H/'firmware';OUT=H/'results'
import argparse
from serial.tools import list_ports
import serial,esptool
from telemetry import WIRE2,FIELDS2
parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--port');parser.add_argument('--skip-backup',action='store_true');args=parser.parse_args()
ports=[p.device for p in list_ports.comports() if p.serial_number=='5A33014245']
PORT=args.port or (ports[0] if len(ports)==1 else None)
assert PORT,'USB/IP device 5A33014245 is not attached to develop'
FW=R/'esp32/balance_car/build/pid'
BASELINE=subprocess.check_output(['git','rev-parse','main'],cwd=R,text=True).strip()
assert subprocess.check_output(['git','branch','--show-current'],cwd=R,text=True).strip()=='codex/test-pid'
OUT.mkdir(parents=True,exist_ok=True);design=json.loads((H/'pid_design.json').read_text())
assert 'WIRELESS_DEBUG_ENABLED = false' in (R/'esp32/balance_car/user_lib/realtime_config.h').read_text()
def open_serial():
 s=serial.Serial(port=None,baudrate=921600,timeout=.03);s.dtr=False;s.rts=False;s.port=PORT;s.open();return s
def receive(s,seconds):
 end=time.monotonic()+seconds;data=b''
 while time.monotonic()<end:data+=s.read(max(1,s.in_waiting))
 return data
with open_serial() as s:
 receive(s,5);s.write(b'STATUS\n');data=receive(s,2)
 match=re.search(rb'STATUS [^\r\n]+',data);assert match,'No preflash STATUS'
 before=match.group().decode();assert 'running=0 ' in before and 'active=0' in before,before
print('PREFLIGHT stopped; no motor commands',flush=True)
base=['--chip','esp32s3','--port',PORT,'--baud','921600','--after','no-reset']
log=io.StringIO()
def command(arguments):
 with contextlib.redirect_stdout(log):esptool.main(arguments)
 (OUT/'flash_tool.log').write_text(log.getvalue(),encoding='utf-8')
command(base+['flash-id']);chip=log.getvalue();assert '34:cd:b0:38:ac:90' in chip.lower() and '8MB' in chip.replace(' ','')
backup=H/'firmware'/('backup_'+datetime.now().strftime('%Y%m%d_%H%M%S'));backup.mkdir(parents=True)
command(base+['read-flash','0x8000','0x1000',str(backup/'partition.bin')])
table=(backup/'partition.bin').read_bytes();entries={}
for i in range(0,3072,32):
 magic,kind,subtype,offset,size,label,flags=struct.unpack('<HBBII16sI',table[i:i+32])
 if magic!=0x50aa:break
 entries[label.split(b'\0')[0].decode()]=(offset,size)
assert entries['factory'][0]==0x10000
assert entries['nvs']==(0x9000,0x6000)
if not args.skip_backup:
 regions=[('factory.bin','0x10000',hex(entries['factory'][1])),('bootloader.bin','0x0','0x8000')]
 if 'otadata' in entries:regions.append(('otadata.bin',hex(entries['otadata'][0]),hex(entries['otadata'][1])))
 for name,offset,size in regions:
  command(base+['read-flash',offset,size,str(backup/name)])
 print('BACKUP_OK existing application/boot partitions; NVS retained',flush=True)
else:
 assert any((p/'factory.bin').exists() for p in (H/'firmware').glob('backup_*')),'No earlier factory backup exists'
 print('REUSING_PRIOR_BACKUP partition check and write/readback verification retained',flush=True)
image=FW/'balance_car.bin';assert image.stat().st_size<entries['factory'][1]
command(base+['write-flash','--flash-mode','dio','--flash-size','8MB','--flash-freq','80m','0x10000',str(image)])
command(base+['read-flash','0x10000',hex(image.stat().st_size),str(OUT/'readback_pid.bin')])
assert hashlib.sha256((OUT/'readback_pid.bin').read_bytes()).digest()==hashlib.sha256(image.read_bytes()).digest(),'Readback mismatch; held in ROM'
if 'otadata' in entries:command(base+['erase-region',hex(entries['otadata'][0]),hex(entries['otadata'][1])])
command(base[:-2]+['--after','hard-reset','verify-flash','0x10000',str(image)])
print('FLASH_READBACK_OK; PID reset into stopped boot',flush=True)
with open_serial() as s:
 boot=receive(s,5);s.write(b'STATUS\nPID\nFAULT\n');replies=receive(s,2)
 text=(boot+replies).decode(errors='replace');(OUT/'boot_and_verify.log').write_text(text,encoding='utf-8')
 statuses=re.findall(r'STATUS [^\r\n]+',text);assert statuses,statuses
 after=statuses[-1];assert all(v in after for v in ['running=0','active=0','fault=0','controller=PID']),after
 assert design['design_sha256'] in text and 'schema=pid_current_v1' in text and 'telemetry=BCU3' in text
 assert 'DEBUG transport=UART wireless=OFF ready=1' in text,text
 s.write(b'SET pitch_kp 24\n');reject=receive(s,.3).decode(errors='replace');assert 'REJECT PID controller' in reject,reject
 s.write(b'UARTSTREAM 200\n');binary=receive(s,2);s.write(b'UARTSTREAM 0\n');receive(s,.3)
 (OUT/'idle_telemetry.bin').write_bytes(binary);frames=[];bad=0;index=0
 while True:
  index=binary.find(b'BCU3',index)
  if index<0 or index+224>len(binary):break
  packet=binary[index:index+224];index+=224
  if zlib.crc32(packet[:220])&0xffffffff!=struct.unpack_from('<I',packet,220)[0]:bad+=1;continue
  frames.append(dict(zip(FIELDS2,WIRE2.unpack(packet[:220]))))
 assert len(frames)>=300 and bad==0,(len(frames),bad)
 assert all(f['mode']==0 and f['fault']==0 and int(f['flags'])&6==0 and f['revision']==int(design['design_sha256'][:8],16) for f in frames)
 assert all(f['command_a']==0 and f['command_b']==0 for f in frames)
record=dict(utc=datetime.now(timezone.utc).isoformat(),port=PORT,mac='34:CD:B0:38:AC:90',main=BASELINE,design=design['design_sha256'],image_sha256=hashlib.sha256(image.read_bytes()).hexdigest(),backup=str(backup),preflash_status=before,postflash_status=after,written_regions=['factory application']+(['OTA selection erase'] if 'otadata' in entries else []),bootloader_preserved=True,partition_preserved=True,nvs_preserved=True,idle_bcu3_frames=len(frames),crc_errors=bad,old_lqi_commands_rejected=True,motor_start_commands_executed=False)
(OUT/'flashed_identity.json').write_text(json.dumps(record,indent=2));assert subprocess.check_output(['git','rev-parse','main'],cwd=R,text=True).strip()==BASELINE
print(json.dumps(record,indent=2));print('PID_FLASH_VERIFIED_STOPPED',flush=True)
