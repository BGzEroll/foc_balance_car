"""Copy only this experiment's source files into the matching remote test branch."""
import base64
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[2]
FILES = [
    "esp32/balance_car/main/CMakeLists.txt",
    "esp32/balance_car/user_lib/balance.h",
    "esp32/balance_car/user_lib/balance.cpp",
    "esp32/balance_car/user_lib/test.cpp",
    "esp32/balance_car/user_lib/hw/motor.h",
    "esp32/balance_car/user_lib/hw/motor.cpp",
]
payload = {p: base64.b64encode((ROOT / p).read_bytes()).decode() for p in FILES}
program = """
import base64,json,pathlib,subprocess,sys
root=pathlib.Path('/home/bgzerol/embedded_project/foc_balance_car')
assert subprocess.check_output(['git','branch','--show-current'],cwd=root,text=True).strip()=='codex/test-upper-controller'
assert subprocess.check_output(['git','rev-parse','main'],cwd=root,text=True).strip()=='9faaf7f22f017444cb60e80a31227c53518281e4'
for name,data in json.load(sys.stdin).items():
    path=root/name
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_bytes(base64.b64decode(data))
print('SYNC_OK test branch only')
"""
encoded = base64.b64encode(program.encode()).decode()
command = "python3 -c 'import base64;exec(base64.b64decode(\"" + encoded + "\"))'"
subprocess.run(["ssh", "develop", command],
               input=json.dumps(payload).encode(), check=True)
