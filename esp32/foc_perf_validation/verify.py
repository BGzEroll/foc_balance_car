"""Verify dev sources and final firmware against the local review artifacts."""
from pathlib import Path
import subprocess,json,hashlib
p=Path(__file__).resolve().parent;root=p.parents[1]
files=['main/CMakeLists.txt','user_lib/app.cpp','user_lib/perf_bench.cpp','user_lib/math_benchmark.inc','user_lib/hw/motor.cpp','user_lib/hw/motor.h','user_lib/hw/motor.lf','user_lib/hw/foc_math.h','user_lib/hw/sensor.cpp','user_lib/hw/sensor.h']
expected={x:hashlib.sha256((root/'esp32/balance_car'/x).read_bytes().replace(b'\r\n',b'\n')).hexdigest() for x in files}
script='''from pathlib import Path
import json,hashlib,subprocess
root=Path('/home/bgzerol/embedded_project/foc_balance_car')
base=root/'esp32/balance_car'
expected=json.loads(PAYLOAD)
actual={x:hashlib.sha256((base/x).read_bytes().replace(b'\\r\\n',b'\\n')).hexdigest() for x in expected}
assert actual==expected,(actual,expected)
assert subprocess.check_output(['git','branch','--show-current'],cwd=root,text=True).strip()=='codex/test-main-foc-perf-240-lut'
main=subprocess.check_output(['git','rev-parse','main'],cwd=root,text=True).strip()
assert main=='a8d1b4fc4aa608fc89710a3e8f1fd8e1b05571e0'
sdk=(base/'sdkconfig').read_text()
assert 'CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ=240' in sdk
assert '# CONFIG_PM_ENABLE is not set' in sdk
data=(base/'build/perf_validation/balance_car.bin').read_bytes()
print(json.dumps({'result':'PASS_SOURCE_AND_FIRMWARE','source_sha256_lf':actual,'main_unchanged':main,'tested_firmware_sha256':hashlib.sha256(data).hexdigest()}))
'''.replace('PAYLOAD',repr(json.dumps(expected)))
result=subprocess.check_output(['ssh','develop','python3 -'],input=script,text=True)
report=json.loads(result)
local=hashlib.sha256((p/'tested_firmware.bin').read_bytes()).hexdigest()
assert local==report['tested_firmware_sha256']=='371835f885ff994f3a4ae3c4eb7db4896a965e6cea86a4c7e39e40cf10617ab3'
assert subprocess.check_output(['git','rev-parse','main'],cwd=root,text=True).strip()==report['main_unchanged']
states=[json.loads(line[7:]) for line in (p/'evidence/final_stopped.log').read_text().splitlines() if line.startswith('STATUS ')]
state=states[-1];assert not state['running'] and not state['requested'] and not state['fault'] and state['aligned']==[0,0]
(p/'evidence/final_stopped.json').write_text(json.dumps(state,indent=2),encoding='utf-8')
(p/'evidence/source_verification.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(report['result'],len(expected),'files; main unchanged; tested binary matches; default stopped.')
