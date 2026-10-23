"""Sync Wi-Fi experiment to develop, guarded against changes to main."""
import base64
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[2]
BRANCH = "codex/test-upper-controller"
BASELINE = "9faaf7f22f017444cb60e80a31227c53518281e4"
assert subprocess.check_output(["git", "branch", "--show-current"], cwd=ROOT, text=True).strip() == BRANCH
assert subprocess.check_output(["git", "rev-parse", "main"], cwd=ROOT, text=True).strip() == BASELINE
files = ["CMakeLists.txt", "user_lib/balance.cpp", "user_lib/balance.h", "sdkconfig", "main/CMakeLists.txt", "user_lib/test.h", "user_lib/test.cpp",
         "user_lib/wifi_debug.h", "user_lib/wifi_debug.cpp", "user_lib/wifi_credentials.h",
         "user_lib/app.cpp", "user_lib/ota.cpp", "user_lib/ota.h", "user_lib/ota_credentials.h",
         "user_lib/hw/motor.cpp", "user_lib/hw/motor.h", "../tuning_preparation/ota_partitions_8mb.csv"]
payload = {f: base64.b64encode((ROOT / "esp32/balance_car" / f).read_bytes()).decode() for f in files}
program = r'''
import base64,json,pathlib,subprocess,sys
root=pathlib.Path('/home/bgzerol/embedded_project/foc_balance_car')
assert subprocess.check_output(['git','branch','--show-current'],cwd=root,text=True).strip()=='codex/test-upper-controller'
assert subprocess.check_output(['git','rev-parse','main'],cwd=root,text=True).strip()=='9faaf7f22f017444cb60e80a31227c53518281e4'
# Keep the secret out of git status even if local ignore changes have not been synced.
exclude=root/'.git/info/exclude'
rule='/esp32/balance_car/user_lib/wifi_credentials.h'
if rule not in exclude.read_text():
    with exclude.open('a') as f: f.write('\n'+rule+'\n')
rule='/esp32/balance_car/user_lib/ota_credentials.h'
if rule not in exclude.read_text():
    with exclude.open('a') as f: f.write('\n'+rule+'\n')
for name,data in json.load(sys.stdin).items():
    p=root/'esp32/balance_car'/name
    p.parent.mkdir(parents=True,exist_ok=True)
    p.write_bytes(base64.b64decode(data))
print('SYNC_OK test branch only; credentials ignored')
'''
encoded = base64.b64encode(program.encode()).decode()
subprocess.run(["ssh", "develop", "python3 -c 'import base64;exec(base64.b64decode(\""+encoded+"\"))'"],
               input=json.dumps(payload).encode(), check=True)
subprocess.run(["ssh", "develop", "bash -c 'source /home/bgzerol/.espressif/tools/activate_idf_v6.0.2.sh >/dev/null && cd /home/bgzerol/embedded_project/foc_balance_car/esp32/balance_car && idf.py -B build/wifi_debug build'"], check=True)
firmware = Path(__file__).resolve().parent / "firmware"
firmware.mkdir(exist_ok=True)
remote_build = "develop:/home/bgzerol/embedded_project/foc_balance_car/esp32/balance_car/build/wifi_debug/"
for name in ["balance_car.bin", "bootloader/bootloader.bin", "partition_table/partition-table.bin"]:
    subprocess.run(["scp", remote_build + name, str(firmware / Path(name).name)], check=True)
subprocess.run(["scp", "develop:/home/bgzerol/embedded_project/foc_balance_car/esp32/balance_car/sdkconfig",
                str(ROOT / "esp32/balance_car/sdkconfig")], check=True)
print("FETCH_OK matching application, bootloader and partition table")
