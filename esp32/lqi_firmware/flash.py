"""Flash directly on develop through an already attached, stopped USB/IP device."""
import argparse,shlex,subprocess
p=argparse.ArgumentParser(description=__doc__);p.add_argument('--port');p.add_argument('--skip-backup',action='store_true');a=p.parse_args()
remote='source /home/bgzerol/.espressif/tools/activate_idf_v6.0.2.sh >/dev/null && cd /home/bgzerol/embedded_project/foc_balance_car && python esp32/lqi_firmware/device_flash.py'
if a.port:remote+=' --port '+shlex.quote(a.port)
if a.skip_backup:remote+=' --skip-backup'
subprocess.run(['ssh','develop','bash -c '+shlex.quote(remote)],check=True)
