"""Project-local SSH/SCP runner. No credentials stored. Requires SSH alias develop."""
from pathlib import Path
import subprocess, sys
ROOT = Path(__file__).resolve().parents[2]
REMOTE = '/home/bgzerol/embedded_project/foc_balance_car'
BRANCH = 'codex/test-main-foc-validation'
EVIDENCE = REMOTE+'/esp32/balance_car/build/main_foc_evidence'

def run(args, **kwargs):
    return subprocess.run(args, check=True, encoding='utf-8', errors='replace', **kwargs)

def ssh(command):
    return run(['ssh', 'develop', command])

def require_test_branch():
    local = subprocess.check_output(['git','branch','--show-current'],cwd=ROOT,text=True).strip()
    remote = subprocess.check_output(['ssh','develop','git -C '+REMOTE+' branch --show-current'],encoding='utf-8').strip()
    assert local == remote == BRANCH, (local,remote,'Refusing to write outside the test branch')

if __name__ == '__main__':
    mode = sys.argv[1]
    if mode in ['sync','build','install-tools','flash']:require_test_branch()
    if mode == 'sync':
        for name in ['main/CMakeLists.txt', 'main/motor.lf', 'sdkconfig', 'user_lib/app.cpp', 'user_lib/foc_bench.cpp',
                     'user_lib/hw/motor.cpp', 'user_lib/hw/motor.h']:
            run(['scp', str(ROOT/'esp32/balance_car'/name), 'develop:'+REMOTE+'/esp32/balance_car/'+name])
    elif mode == 'build':
        ssh('. /home/bgzerol/.espressif/tools/activate_idf_v6.0.2.sh >/dev/null 2>&1; '
            'cd '+REMOTE+'/esp32/balance_car; idf.py -B build/main_foc_validation build '
            '> /tmp/main_foc_build.log 2>&1; result=$?; tail -35 /tmp/main_foc_build.log; exit $result')
    elif mode == 'install-tools':
        ssh('mkdir -p '+EVIDENCE)
        for name in ['run_new.py','capture.py','gate.py','check_modulation.py']:
            run(['scp',str(Path(__file__).parent/name),'develop:'+EVIDENCE+'/'+name])
    elif mode == 'flash':
        ssh('test -s '+EVIDENCE+'/before_flash.bin && '
            'cd '+REMOTE+'/esp32/balance_car && '
            '/home/bgzerol/.espressif/tools/python/v6.0.2/venv/bin/python3 -m esptool '
            '--chip esp32s3 --port /dev/serial/by-id/usb-1a86_USB_Single_Serial_5A33014245-if00 '
            '--baud 460800 write-flash 0x10000 build/main_foc_validation/balance_car.bin')
    elif mode == 'collect':
        dest=Path(__file__).parent/'evidence';dest.mkdir(exist_ok=True)
        for extension in ['json','csv','log','sha256','txt']:
            run(['scp','develop:'+EVIDENCE+'/*.'+extension,str(dest)])
        run(['scp','develop:'+EVIDENCE+'/final_tested.bin',str(Path(__file__).parent/'tested_firmware.bin')])
    else:
        ssh(sys.argv[2])
