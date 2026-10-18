from pathlib import Path
import sys,time,json,subprocess,serial,hashlib
root=Path(__file__).resolve().parent
project=root.parent.parent
device='/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A33014245-if00'
label=sys.argv[1]
seconds=float(sys.argv[2]) if len(sys.argv)>2 else 25
image=project/'build/balance_car.bin'
tool=[sys.executable,'-m','esptool','--chip','esp32s3','--port',device,'--baud','460800']
subprocess.run(tool+['write-flash','0x10000',str(image)],check=True)
data=bytearray()
with serial.Serial(device,115200,timeout=.2) as port:
 port.dtr=False;port.rts=True;time.sleep(.1);port.rts=False
 until=time.monotonic()+seconds
 while time.monotonic()<until:data.extend(port.read(16384))
(root/f'{label}.log').write_bytes(data)
(root/f'{label}.bin').write_bytes(image.read_bytes())
text=data.decode('utf-8',errors='replace')
frames=[]
for line in text.splitlines():
 if line.startswith('YIELD_JSON '):
  try:frames.append(json.loads(line[len('YIELD_JSON '):]))
  except ValueError:pass
bad=[x for x in ('Guru Meditation','assert failed','watchdog','abort()','panic') if x.lower() in text.lower()]
result={'label':label,'duration_s':seconds,'sha256':hashlib.sha256(image.read_bytes()).hexdigest(),'failure_markers':bad,'frames':frames}
(root/f'{label}.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
print('CAPTURE',label,'frames',len(frames),'failure_markers',bad)
if frames:print(json.dumps(frames[-1],ensure_ascii=False))
else:print(text[-6000:])
if bad:print(text[text.lower().find(bad[0].lower())-150:text.lower().find(bad[0].lower())+1800])
sys.exit(1 if bad or not frames else 0)
