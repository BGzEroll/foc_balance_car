"""Back up and install diagnostic sources only; run inside the remote build directory."""
from pathlib import Path
import shutil,hashlib
r=Path(__file__).resolve().parent;p=r.parent.parent;b=r/'before';b.mkdir(exist_ok=True)
for rel in ['user_lib/foc_current.cpp','user_lib/hw/sensor.cpp','user_lib/hw/sensor.h','build/balance_car.bin','build/balance_car.elf']:
 f=p/rel;dst=b/f.name
 if not dst.exists():shutil.copy2(f,dst)
 print(rel,hashlib.sha256(f.read_bytes()).hexdigest())
assert hashlib.sha256((b/'foc_current.cpp').read_bytes()).hexdigest()=='a3ca809fa2a8ea6f3fe81d160bbd2c6148a42a39b1d5779851518185498d8c32'
for src,dst in [('foc_sensor_recorder.cpp','user_lib/foc_current.cpp'),('sensor.cpp','user_lib/hw/sensor.cpp'),('sensor.h','user_lib/hw/sensor.h'),('sensor_record.inc','user_lib/sensor_record.inc')]:shutil.copy2(r/src,p/dst)
