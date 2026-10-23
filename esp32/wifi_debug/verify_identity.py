"""Verify local/remote test sources and flashed binary without touching main."""
import base64
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
parser = argparse.ArgumentParser()
parser.add_argument("--record-ota", action="store_true", help="Record new application only after authenticated live identity check")
parser.add_argument("--host", default="192.168.114.243")
args = parser.parse_args()
names = ["CMakeLists.txt", "sdkconfig", "user_lib/balance.cpp", "user_lib/balance.h", "main/CMakeLists.txt", "user_lib/test.cpp", "user_lib/test.h", "user_lib/realtime_config.h",
         "user_lib/wifi_debug.cpp", "user_lib/wifi_debug.h", "user_lib/app.cpp", "user_lib/ota.cpp", "user_lib/ota.h",
         "user_lib/hw/motor.cpp", "user_lib/hw/motor.h", "../tuning_preparation/ota_partitions_8mb.csv"]
local = {name: hashlib.sha256((ROOT / "esp32/balance_car" / name).read_bytes()).hexdigest() for name in names}
program = """
import hashlib,json,pathlib,subprocess,sys
root=pathlib.Path('/home/bgzerol/embedded_project/foc_balance_wifi_test')
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
if args.record_ota:
    sys.path.insert(0, str(ROOT / "esp32"))
    from ota.client import OTA, image_info
    running = OTA(args.host).request("status")
    image = image_info((HERE / "firmware/balance_car.bin").read_bytes())
    assert running["elf_sha256"] == image["elf_sha256"] and running["healthy"]
    assert not running["pending_verify"] and not running["locked"] and running["state"] == 2
    # Identity verification is read-only. The continuous logger owns the single
    # TCP debug connection; opening another subscriber would disrupt that run.
    # OTA boot confirmation already checked TCP/UDP before releasing its lock.
    for name, digest in remote["binaries"].items():
        assert hashlib.sha256((HERE / "firmware" / name).read_bytes()).hexdigest() == digest
    identity.update(remote["binaries"])
    identity.update(application=image, ota_running=running)
for name, digest in remote["binaries"].items():
    assert identity[name] == digest, "Built/flashed binary mismatch: " + name
config = dict(line.split("=", 1) for line in (ROOT / "esp32/balance_car/sdkconfig").read_text(encoding="utf-8").splitlines()
              if line.startswith("CONFIG_") and "=" in line)
keys = ["CONFIG_LWIP_TCPIP_TASK_PRIO", "CONFIG_LWIP_TCPIP_TASK_AFFINITY", "CONFIG_LWIP_TCPIP_CORE_LOCKING",
        "CONFIG_LWIP_CHECK_THREAD_SAFETY", "CONFIG_LWIP_IRAM_OPTIMIZATION",
        "CONFIG_ESP_WIFI_DYNAMIC_TX_BUFFER_NUM", "CONFIG_ESP_WIFI_TX_BA_WIN",
        "CONFIG_ESP32S3_INSTRUCTION_CACHE_SIZE", "CONFIG_ESP32S3_INSTRUCTION_CACHE_WRAP",
        "CONFIG_ESPTOOLPY_FLASHSIZE", "CONFIG_PARTITION_TABLE_CUSTOM_FILENAME", "CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE"]
for key in keys:
    assert remote["config"].get(key) == config.get(key), "SDK config mismatch: " + key
assert subprocess.check_output(["git", "branch", "--show-current"], cwd=ROOT, text=True).strip() == identity["branch"]
assert subprocess.check_output(["git", "rev-parse", "main"], cwd=ROOT, text=True).strip() == identity["main_before"]
identity.update(source_sha256=local, remote_source_verified=True, remote_binary_verified=True)
identity["validated_sdkconfig"] = {key: config.get(key) for key in keys}
identity_path.write_text(json.dumps(identity, indent=2), encoding="utf-8")
print("IDENTITY_OK local/remote test sources and flashed binary; main unchanged")
