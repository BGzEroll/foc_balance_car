"""Export complete CSV blocks from the persistent serial session."""
from pathlib import Path
import argparse
import csv
import io
import json
import re
import subprocess

folder = Path(__file__).resolve().parent / "results"
parser = argparse.ArgumentParser()
parser.add_argument("--prefix", default="live")
args = parser.parse_args()
result = subprocess.run(["ssh", "develop", "cat /home/bgzerol/embedded_project/foc_balance_car/esp32/balance_car/build/upper_validation/live.log"],
                        stdout=subprocess.PIPE, check=True)
text = result.stdout.decode(errors="replace").replace("\r", "")
(folder / "live.log").write_text(text)
name = args.prefix
block = []
collecting = False
occurrences = {}
metadata = {}
tuning = None
windows = {}
for line in text.splitlines():
    if line.startswith("COMMAND ") and len(line.split()) > 1 and line.split()[1] in ("SHADOW", "BALANCE", "PROBE"):
        name = args.prefix + "_" + re.sub(r"[^a-zA-Z0-9_-]", "_", line[8:]).lower()
        occurrences[name] = occurrences.get(name, 0) + 1
        if occurrences[name] > 1:
            name += "_" + str(occurrences[name])
    elif line.startswith("RUN accepted=") and "tuning=" in line:
        tuning = [float(x) for x in line.split("tuning=", 1)[1].split()[0].split(",")]
    elif line.startswith("TUNING "):
        tuning = [float(value) for value in re.findall(r"=(\d+\.\d+)", line)]
    elif line.startswith("CSV_BEGIN"):
        block = []
        collecting = True
        metadata = {key: int(value) for key, value in re.findall(r"(\w+)=(\d+)", line)}
        if tuning is not None:
            metadata["velocity_kp_velocity_ki_pitch_kd"] = tuning
    elif line == "CSV_END" and collecting:
        parsed = list(csv.reader(io.StringIO("\n".join(block))))
        valid = bool(parsed) and all(len(row) == len(parsed[0]) for row in parsed)
        if valid:
            output_name = name
            if "_balance_0_" in name:
                windows[name] = windows.get(name, 0) + 1
                output_name += "_window_" + str(windows[name])
            (folder / (output_name + ".csv")).write_text("\n".join(block) + "\n")
            (folder / (output_name + ".json")).write_text(json.dumps(metadata, indent=2) + "\n")
            print(output_name, "rows", len(block) - 1)
        else:
            (folder / (name + ".csv")).unlink(missing_ok=True)
            print(name, "SKIPPED malformed block")
        collecting = False
    elif collecting:
        block.append(line)
