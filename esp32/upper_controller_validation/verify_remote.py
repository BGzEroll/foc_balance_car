"""Check source parity and preserve firmware identity without touching hardware."""
import base64
import hashlib
import json
from pathlib import Path
import subprocess

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
FILES = ["esp32/balance_car/" + p for p in [
    "main/CMakeLists.txt", "user_lib/balance.cpp", "user_lib/balance.h",
    "user_lib/test.cpp", "user_lib/hw/motor.cpp", "user_lib/hw/motor.h"]]
program = """
import hashlib,json,pathlib,subprocess
root=pathlib.Path('/home/bgzerol/embedded_project/foc_balance_car')
def git(*args):return subprocess.check_output(['git',*args],cwd=root,text=True).strip()
files=FILES
data=dict(branch=git('branch','--show-current'),main=git('rev-parse','main'),files={})
for name in files:
    data['files'][name]=hashlib.sha256((root/name).read_bytes()).hexdigest()
for name in ['balance_car.bin','balance_car.elf','pre_upper_flash_2MB.bin']:
    data[name+'_sha256']=hashlib.sha256((root/'esp32/balance_car/build/upper_validation'/name).read_bytes()).hexdigest()
print(json.dumps(data))
""".replace("FILES", repr(FILES))
encoded = base64.b64encode(program.encode()).decode()
command = "python3 -c 'import base64;exec(base64.b64decode(\"" + encoded + "\"))'"
data = json.loads(subprocess.check_output(["ssh", "develop", command]))
assert data["branch"] == "codex/test-upper-controller"
assert data["main"] == "9faaf7f22f017444cb60e80a31227c53518281e4"
assert subprocess.check_output(["git", "rev-parse", "main"], cwd=ROOT, text=True).strip() == data["main"]
for name in FILES:
    assert hashlib.sha256((ROOT / name).read_bytes()).hexdigest() == data["files"][name], name
data["source_parity"] = True
(HERE / "results/build_identity.json").write_text(json.dumps(data, indent=2) + "\n")
print(json.dumps(data, indent=2))
