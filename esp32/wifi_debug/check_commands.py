"""Exercise real TCP commands without calibration or motor activation."""
import json
from pathlib import Path
import sys
from client import Client

HERE = Path(__file__).resolve().parent
c = Client(sys.argv[1])
results = {}
try:
    status = c.command("STATUS")
    assert "running=0" in status and "active=0" in status
    results["status_before"] = status
    results["ping"] = c.command("PING command_check", "PONG")
    # Reapply existing B defaults; no current output is requested.
    results["tuning"] = c.command("TUNE 0.5 0.2 0.05")
    for name, command in [("nonfinite_tuning", "TUNE nan 0.2 0.05"),
                          ("oversized_line", "X" * 120),
                          ("unsupported_rate", "STREAM UDP 201"),
                          ("network_dump", "DUMP")]:
        try:
            c.command(command)
        except RuntimeError as e:
            assert str(e).startswith("REJECT")
            results[name] = str(e)
        else:
            raise AssertionError(name + " was unexpectedly accepted")
    results["stop"] = c.command("STOP")
    results["status_after"] = c.command("STATUS")
    assert "running=0" in results["status_after"] and "active=0" in results["status_after"]
    results["network"] = c.command("NET")
finally:
    c.close()
(HERE / "results/command_checks.json").write_text(json.dumps(results, indent=2), encoding="utf-8")
print(json.dumps(results, indent=2))
