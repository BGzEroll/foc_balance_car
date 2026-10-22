"""Compare the portable C++ controller with the unchanged MATLAB controller."""
import base64
import csv
import io
import json
from pathlib import Path
import subprocess

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
OUTPUT = ROOT / "esp32/balance_car/build/upper_controller"
payload = {p.name: base64.b64encode(p.read_bytes()).decode() for p in [
    HERE / "native.cpp", ROOT / "esp32/balance_car/user_lib/balance.cpp",
    ROOT / "esp32/balance_car/user_lib/balance.h", OUTPUT / "inputs.csv", OUTPUT / "adapter_inputs.csv"]}
program = """
import base64,json,pathlib,subprocess,sys,tempfile
with tempfile.TemporaryDirectory(prefix='upper_native_') as folder:
    root=pathlib.Path(folder)
    for name,data in json.load(sys.stdin).items():
        (root/name).write_bytes(base64.b64decode(data))
    subprocess.run(['g++','-O2','-std=c++17','-Wall','-Wextra','-Werror','native.cpp','balance.cpp','-o','native'],cwd=root,check=True)
    controller=subprocess.check_output([str(root/'native')],input=(root/'inputs.csv').read_bytes())
    adapter=subprocess.check_output([str(root/'native'),'observe'],input=(root/'adapter_inputs.csv').read_bytes())
    print(json.dumps(dict(controller=controller.decode(),adapter=adapter.decode())))
"""
encoded = base64.b64encode(program.encode()).decode()
command = "python3 -c 'import base64;exec(base64.b64decode(\"" + encoded + "\"))'"
result = subprocess.run(["ssh", "develop", command], input=json.dumps(payload).encode(),
                        stdout=subprocess.PIPE, check=True)
data = json.loads(result.stdout)
(OUTPUT / "native.csv").write_text(data["controller"])
expected = list(csv.reader(io.StringIO((OUTPUT / "expected.csv").read_text())))
actual = list(csv.reader(io.StringIO(data["controller"])))
assert len(actual) == len(expected) == 14400
assert all(len(row) == 5 for row in actual + expected)
errors = [max(abs(float(a[i]) - float(e[i])) for a, e in zip(actual, expected)) for i in range(5)]
summary = json.loads((OUTPUT / "reference.json").read_text())
summary["max_abs_error"] = dict(zip(["left_current_A", "right_current_A", "target_speed_m_s",
                                     "target_pitch_rad", "distance_m"], errors))
summary["tolerance"] = [0.0002, 0.0002, 0.00002, 0.00002, 0.00002]
summary["pass"] = all(e < t for e, t in zip(errors, summary["tolerance"]))
adapter_expected = list(csv.reader(io.StringIO((OUTPUT / "adapter_expected.csv").read_text())))
adapter_actual = list(csv.reader(io.StringIO(data["adapter"])))
assert len(adapter_actual) == len(adapter_expected) == 6000
adapter_errors = [max(abs(float(a[i]) - float(e[i])) for a, e in
                     zip(adapter_actual, adapter_expected)) for i in range(5)]
summary["adapter_max_abs_error"] = dict(zip(
    ["speed_m_s", "pitch_rad", "pitch_rate_rad_s", "yaw_rad", "yaw_rate_rad_s"], adapter_errors))
summary["adapter_pass"] = all(e < 0.00001 for e in adapter_errors)
summary["pass"] = summary["pass"] and summary["adapter_pass"]
(HERE / "results").mkdir(exist_ok=True)
(HERE / "results/native_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
print(json.dumps(summary, indent=2))
assert summary["pass"]
