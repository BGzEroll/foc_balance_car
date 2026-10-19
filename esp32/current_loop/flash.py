from pathlib import Path
import subprocess,sys,hashlib
r=Path(__file__).resolve().parent;project=r.parent.parent
tool=[sys.executable,'-m','esptool','--chip','esp32s3','--port','/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A33014245-if00','--baud','460800']
image=project/'build/balance_car.bin'
assert (project/'build/current_before/firmware.bin').exists(),'Missing prior firmware backup'
subprocess.run(tool+['write-flash','0x10000',str(image)],check=True)
(r/'tested_firmware.bin').write_bytes(image.read_bytes())
print('SHA256',hashlib.sha256(image.read_bytes()).hexdigest())
