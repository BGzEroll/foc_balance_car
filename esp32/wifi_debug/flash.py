"""Flash via local USB; back up existing 2 MB first. Motors stay stopped."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / ".tools"))
import esptool

parser = argparse.ArgumentParser()
parser.add_argument("--port", default="COM10")
args = parser.parse_args()
root = HERE.parents[1]
assert subprocess.check_output(["git", "branch", "--show-current"], cwd=root, text=True).strip() == "codex/test-upper-controller"
before = subprocess.check_output(["git", "rev-parse", "main"], cwd=root, text=True).strip()
assert before == "9faaf7f22f017444cb60e80a31227c53518281e4"
configuration=(root / "esp32/balance_car/sdkconfig").read_text(encoding="utf-8")
assert 'CONFIG_PARTITION_TABLE_SINGLE_APP=y' in configuration and 'CONFIG_ESPTOOLPY_FLASHSIZE_2MB=y' in configuration, "OTA migration requires its reviewed, separate USB procedure"
fw = HERE / "firmware"
backup = fw / "pre_wifi_2MB.bin"
if not backup.exists():
    esptool.main(["--chip", "esp32s3", "--port", args.port, "--baud", "460800",
                  "read-flash", "0", "0x200000", str(backup)])
esptool.main(["--chip", "esp32s3", "--port", args.port, "--baud", "460800", "write-flash",
              "--flash-mode", "dio", "--flash-size", "2MB", "--flash-freq", "80m",
              "0x0", str(fw / "bootloader.bin"), "0x8000", str(fw / "partition-table.bin"),
              "0x10000", str(fw / "balance_car.bin")])
assert subprocess.check_output(["git", "rev-parse", "main"], cwd=root, text=True).strip() == before
identity = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in fw.glob("*.bin")}
identity.update(branch="codex/test-upper-controller", main_before=before, main_after=before)
identity["source_sha256"] = {
    str(p.relative_to(root)).replace("\\", "/"): hashlib.sha256(p.read_bytes()).hexdigest()
    for p in [root / "esp32/balance_car/main/CMakeLists.txt",
              *(root / "esp32/balance_car/user_lib" / name for name in
                ["test.cpp", "test.h", "balance.cpp", "balance.h", "wifi_debug.cpp", "wifi_debug.h"])]
}
(HERE / "results").mkdir(exist_ok=True)
(HERE / "results/build_identity.json").write_text(json.dumps(identity, indent=2), encoding="utf-8")
