"""Summarize recorded stand and supported ground trials; no hardware access."""
import csv
import json
import math
from pathlib import Path
import statistics

HERE = Path(__file__).resolve().parent
FOLDER = HERE / "results"
CASES = {
    "ma_forward": ("stand_probe", [0]),
    "ma_forward_025": ("stand_probe", [0]),
    "mb_forward_025": ("stand_probe", [1]),
    "ma_forward_045": ("stand_probe", [0]),
    "ma_reverse_045": ("stand_probe", [0]),
    "mb_reverse_025": ("stand_probe", [1]),
    "live_shadow_1000_0_7": ("stand_shadow", []),
    "live_balance_80_0_45": ("stand_balance", [0, 1]),
    "live_shadow_1000_0_7_2": ("ground_supported_shadow", []),
    "live_balance_200_0_7": ("ground_supported_balance", [0, 1]),
    "live_balance_500_0_7": ("ground_light_support_balance", [0, 1]),
    "live_balance_2000_0_7": ("ground_light_support_balance", [0, 1]),
    "live_balance_10000_0_7": ("ground_light_support_balance", [0, 1]),
    "live_shadow_10000_0_7": ("stand_shadow", []),
    "tuning_balance_10000_0_7": ("ground_tuning_a", [0, 1]),
    "tuning_balance_10000_0_7_2": ("ground_tuning_b", [0, 1]),
    "continuous_balance_0_0_7_window_1": ("continuous_tuning_a", [0, 1]),
    "continuous_balance_0_0_7_window_2": ("continuous_tuning_b", [0, 1]),
}
summary = {}
for name, (phase, motors) in CASES.items():
    path = FOLDER / (name + ".csv")
    if not path.exists():
        continue
    rows = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(path.open())]
    assert rows and all(all(math.isfinite(v) for v in row.values()) for row in rows)
    result = dict(phase=phase, samples=len(rows), last_sample_us=rows[-1]["t_us"],
                  dt_us_min=min(r["dt_us"] for r in rows),
                  dt_us_max=max(r["dt_us"] for r in rows),
                  calculate_us_max=max(r["calculate_us"] for r in rows),
                  saturation_fraction=statistics.mean(r["saturated"] for r in rows),
                  pitch_deg_initial=math.degrees(rows[0]["pitch"]),
                  pitch_deg_final=math.degrees(rows[-1]["pitch"]),
                  pitch_deg_min=math.degrees(min(r["pitch"] for r in rows)),
                  pitch_deg_max=math.degrees(max(r["pitch"] for r in rows)))
    steady = [r for r in rows if r["t_us"] >= rows[0]["t_us"] + 2000000] or rows
    result["pitch_rms_deg_steady"] = math.degrees(math.sqrt(statistics.mean(r["pitch"] ** 2 for r in steady)))
    result["saturation_fraction_steady"] = statistics.mean(r["saturated"] for r in steady)
    result["distance_final_m"] = rows[-1]["distance"]
    metadata = path.with_suffix(".json")
    if metadata.exists():
        result["run_metadata"] = json.loads(metadata.read_text())
    for index in motors:
        motor = "a" if index == 0 else "b"
        sign = -1 if index == 0 else 1
        samples = [r for r in rows if r["t_us"] >= 20000 and
                   r.get("current_age_" + motor + "_us", 0) < 1500]
        errors = [r["iq_" + motor] - sign * r["cmd_" + motor] for r in samples]
        result[motor] = dict(
            forward_speed_min=min(r["forward_" + motor] for r in rows),
            forward_speed_max=max(r["forward_" + motor] for r in rows),
            same_snapshot_current_error_rms_A=math.sqrt(statistics.mean(e * e for e in errors)))
        if len(rows) > 1 and max(r["t_us"] - p["t_us"] for p, r in zip(rows, rows[1:])) < 10000:
            delayed = [rows[i]["iq_" + motor] - sign * rows[i - 1]["cmd_" + motor]
                       for i in range(1, len(rows)) if rows[i]["t_us"] >= 20000]
            result[motor]["previous_command_tracking_rms_A"] = math.sqrt(statistics.mean(e * e for e in delayed))
        if "angle_" + motor in rows[0]:
            result[motor]["forward_displacement_rad"] = sign * (
                rows[-1]["angle_" + motor] - rows[0]["angle_" + motor])
    summary[name] = result
live = (FOLDER / "live.log").read_text()
summary["final_session_watchdog_message"] = "watchdog" in live.lower()
(FOLDER / "hardware_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
print(json.dumps(summary, indent=2))
