"""Verify local/remote test sources and flashed binary without touching main."""
import base64
import hashlib
import json
from pathlib import Path
import subprocess

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
names = ["main/CMakeLists.txt", "user_lib/test.cpp", "user_lib/test.h",
         "user_lib/wifi_debug.cpp", "user_lib/wifi_debug.h"]
local = {name: hashlib.sha256((ROOT / "esp32/balance_car" / name).read_bytes()).hexdigest() for name in names}
program = """
import hashlib,json,pathlib,subprocess,sys
root=pathlib.Path('/home/bgzerol/embedded_project/foc_balance_car')
assert subprocess.check_output(['git','branch','--show-current'],cwd=root,text=True).strip()=='codex/test-upper-controller'
assert subprocess.check_output(['git','rev-parse','main'],cwd=root,text=True).strip()=='9faaf7f22f017444cb60e80a31227c53518281e4'
project=root/'esp32/balance_car'
print(json.dumps(dict(source_sha256={n:hashlib.sha256((project/n).read_bytes()).hexdigest() for n in json.load(sys.stdin)},
 binaries={n.split('/')[-1]:hashlib.sha256((project/'build/wifi_debug'/n).read_bytes()).hexdigest()
 for n in ['balance_car.bin','bootloader/bootloader.bin','partition_table/partition-table.bin']},
 config=dict(line.split('=',1) for line in (project/'sdkconfig').read_text().splitlines() if line.startswith('CONFIG_') and '=' in line))))
"""
encoded = base64.b64encode(program.encode()).decode()
result = subprocess.run(["ssh", "develop", "python3 -c 'import base64;exec(base64.b64decode(\"" + encoded + "\"))'"],
                        input=json.dumps(names).encode(), capture_output=True, check=True)
remote = json.loads(result.stdout)
assert remote["source_sha256"] == local, "Local/remote source mismatch"
identity_path = HERE / "results/build_identity.json"
identity = json.loads(identity_path.read_text(encoding="utf-8"))
for name, digest in remote["binaries"].items():
    assert identity[name] == digest, "Built/flashed binary mismatch: " + name
config = dict(line.split("=", 1) for line in (ROOT / "esp32/balance_car/sdkconfig").read_text(encoding="utf-8").splitlines()
              if line.startswith("CONFIG_") and "=" in line)
keys = ["CONFIG_LWIP_TCPIP_TASK_AFFINITY", "CONFIG_LWIP_TCPIP_CORE_LOCKING",
        "CONFIG_LWIP_CHECK_THREAD_SAFETY", "CONFIG_LWIP_IRAM_OPTIMIZATION",
        "CONFIG_ESP_WIFI_DYNAMIC_TX_BUFFER_NUM", "CONFIG_ESP_WIFI_TX_BA_WIN",
        "CONFIG_ESP32S3_INSTRUCTION_CACHE_SIZE", "CONFIG_ESP32S3_INSTRUCTION_CACHE_WRAP"]
for key in keys:
    assert remote["config"].get(key) == config.get(key), "SDK config mismatch: " + key
assert subprocess.check_output(["git", "branch", "--show-current"], cwd=ROOT, text=True).strip() == identity["branch"]
assert subprocess.check_output(["git", "rev-parse", "main"], cwd=ROOT, text=True).strip() == identity["main_before"]
identity.update(source_sha256=local, remote_source_verified=True, remote_binary_verified=True)
identity["validated_sdkconfig"] = {key: config.get(key) for key in keys}
identity_path.write_text(json.dumps(identity, indent=2), encoding="utf-8")
print("IDENTITY_OK local/remote test sources and flashed binary; main unchanged")
