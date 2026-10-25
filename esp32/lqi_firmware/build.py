"""Sync to develop's primary test branch and build there; never fetch firmware bins."""
import base64,hashlib,json,subprocess
from pathlib import Path
H=Path(__file__).resolve().parent;R=H.parents[1];P=R/'esp32/balance_car';BASE=subprocess.check_output(['git','rev-parse','main'],cwd=R,text=True).strip()
assert subprocess.check_output(['git','branch','--show-current'],cwd=R,text=True).strip()=='codex/test-pid'
assert subprocess.check_output(['git','rev-parse','main'],cwd=R,text=True).strip()==BASE
paths=[P/'CMakeLists.txt',P/'sdkconfig',P/'main/CMakeLists.txt',P/'main/main.c',R/'.gitignore',R/'esp32/tuning_preparation/ota_partitions_8mb.csv',H/'pid_design.json',H/'device_flash.py',H/'telemetry.py']
paths += [p for p in (P/'user_lib').rglob('*') if p.is_file() and p.suffix in ['.cpp','.h','.lf']]
payload={p.relative_to(R).as_posix():base64.b64encode(p.read_bytes()).decode() for p in paths}
program="""
import base64,json,pathlib,subprocess,sys
r=pathlib.Path('/home/bgzerol/embedded_project/foc_balance_car')
assert subprocess.check_output(['git','branch','--show-current'],cwd=r,text=True).strip()=='codex/test-pid'
main_before=subprocess.check_output(['git','rev-parse','main'],cwd=r,text=True).strip()
for name,value in json.load(sys.stdin).items():
 p=r/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(base64.b64decode(value))
assert subprocess.check_output(['git','rev-parse','main'],cwd=r,text=True).strip()==main_before
print('SYNC_OK primary test branch, main unchanged')
"""
enc=base64.b64encode(program.encode()).decode();subprocess.run(['ssh','develop',"python3 -c 'import base64;exec(base64.b64decode(\""+enc+"\"))'"],input=json.dumps(payload).encode(),check=True)
subprocess.run(['ssh','develop',"bash -c 'source /home/bgzerol/.espressif/tools/activate_idf_v6.0.2.sh >/dev/null && cd /home/bgzerol/embedded_project/foc_balance_car/esp32/balance_car && idf.py -B build/pid build'"],check=True)
print('REMOTE_BUILD_OK binaries remain on develop; run flash.py after stopped USB/IP handoff')
