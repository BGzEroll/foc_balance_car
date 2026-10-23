"""Flash stopped, UART-only test firmware over USB. Never CAL or BALANCE."""
import hashlib
import io
import contextlib
import json
import struct
import subprocess
import sys
from pathlib import Path

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
FW=HERE.parent/'wifi_debug/firmware'
sys.path.insert(0,str(HERE.parent/'wifi_debug/.tools'))
import esptool

baseline='9faaf7f22f017444cb60e80a31227c53518281e4'
assert subprocess.check_output(['git','branch','--show-current'],text=True,cwd=ROOT).strip()=='codex/test-upper-controller'
assert subprocess.check_output(['git','rev-parse','main'],text=True,cwd=ROOT).strip()==baseline
assert 'WIRELESS_DEBUG_ENABLED = false' in (ROOT/'esp32/balance_car/user_lib/realtime_config.h').read_text()
base=['--chip','esp32s3','--port','COM10','--baud','921600','--after','no-reset']
output=io.StringIO()
with contextlib.redirect_stdout(output):esptool.main(base+['flash-id'])
identity=output.getvalue()
assert '34:cd:b0:38:ac:90' in identity.lower() and '8MB' in identity.replace(' ','')
print('CHIP_OK expected car; ROM bootloader holds outputs stopped',flush=True)
table=FW/'pre_uart_partition_table.bin'
esptool.main(base+['read-flash','0x8000','0x1000',str(table)])
entries={}
for i in range(0,3072,32):
 magic,kind,subtype,offset,size,label,flags=struct.unpack('<HBBII16sI',table.read_bytes()[i:i+32])
 if magic!=0x50aa:break
 entries[label.split(b'\0')[0].decode()]=(offset,size)
assert entries['factory']==(0x10000,0x180000)
assert entries['otadata']==(0x190000,0x2000)
assert entries['nvs']==(0x9000,0x6000)
image=FW/'balance_car.bin';assert 1024<image.stat().st_size<entries['factory'][1]
esptool.main(base+['read-flash','0x190000','0x2000',str(FW/'pre_uart_otadata.bin')])
# Clear only OTA selection so the newly written factory app boots. NVS and
# the two existing OTA application slots are retained.
esptool.main(base+['erase-region','0x190000','0x2000'])
esptool.main(base[:-2]+['--after','hard-reset','write-flash','--flash-mode','dio',
 '--flash-size','8MB','--flash-freq','80m','0x0',str(FW/'bootloader.bin'),'0x10000',str(image)])
assert subprocess.check_output(['git','rev-parse','main'],text=True,cwd=ROOT).strip()==baseline
record=dict(port='COM10',mac='34:CD:B0:38:AC:90',transport='UART',wireless=False,
 image_sha256=hashlib.sha256(image.read_bytes()).hexdigest(),main=baseline,
 written_regions=['factory application','OTA boot selection'],nvs_preserved=True)
(HERE/'results/uart_only_flash.json').write_text(json.dumps(record,indent=2))
print('UART_ONLY_FLASH_OK main unchanged; no motor start',flush=True)
