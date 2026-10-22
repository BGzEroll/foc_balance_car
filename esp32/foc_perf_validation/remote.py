from pathlib import Path
import subprocess,sys
ROOT=Path(__file__).resolve().parents[2]
REMOTE='/home/bgzerol/embedded_project/foc_balance_car'
BRANCH='codex/test-main-foc-perf-240-lut'
EVIDENCE=REMOTE+'/esp32/balance_car/build/perf_evidence'
def run(args,**kw): return subprocess.run(args,check=True,**kw)
def ssh(cmd): return run(['ssh','develop',cmd])
def guard():
    assert subprocess.check_output(['git','branch','--show-current'],cwd=ROOT,text=True).strip()==BRANCH
    assert subprocess.check_output(['ssh','develop','git -C '+REMOTE+' branch --show-current'],text=True).strip()==BRANCH
mode=sys.argv[1]
if mode in ['sync','build','backup','flash','tools']:guard()
if mode=='sync':
    for file in ['main/CMakeLists.txt','sdkconfig','user_lib/app.cpp','user_lib/perf_bench.cpp','user_lib/math_benchmark.inc','user_lib/hw/motor.cpp','user_lib/hw/motor.h','user_lib/hw/motor.lf','user_lib/hw/foc_math.h']:
        run(['scp',str(ROOT/'esp32/balance_car'/file),'develop:'+REMOTE+'/esp32/balance_car/'+file])
elif mode=='build':
    ssh('. /home/bgzerol/.espressif/tools/activate_idf_v6.0.2.sh >/dev/null 2>&1 && cd '+REMOTE+'/esp32/balance_car && idf.py -B build/perf_validation build > /tmp/perf_build.log 2>&1; result=$?; tail -18 /tmp/perf_build.log; exit $result')
elif mode=='tools':
    ssh('mkdir -p '+EVIDENCE)
    for file in ['run_new.py','run_reference.py','capture.py','gate.py','reference_v18.bin']:
        run(['scp',str(Path(__file__).parent/file),'develop:'+EVIDENCE+'/'+file])
elif mode=='backup':
    ssh('mkdir -p '+EVIDENCE+' && test ! -e '+EVIDENCE+'/before_flash.bin && /home/bgzerol/.espressif/tools/python/v6.0.2/venv/bin/python3 -m esptool --chip esp32s3 --port /dev/serial/by-id/usb-1a86_USB_Single_Serial_5A33014245-if00 --baud 460800 read-flash 0 0x200000 '+EVIDENCE+'/before_flash.bin > '+EVIDENCE+'/backup.log 2>&1 && sha256sum '+EVIDENCE+'/before_flash.bin')
elif mode=='flash':
    label=sys.argv[2]
    assert label.replace('_','').isalnum()
    ssh('test -s '+EVIDENCE+'/before_flash.bin && cd '+REMOTE+'/esp32/balance_car && cp build/perf_validation/balance_car.bin '+EVIDENCE+'/'+label+'.bin && cp build/perf_validation/balance_car.elf '+EVIDENCE+'/'+label+'.elf && /home/bgzerol/.espressif/tools/python/v6.0.2/venv/bin/python3 -m esptool --chip esp32s3 --port /dev/serial/by-id/usb-1a86_USB_Single_Serial_5A33014245-if00 --baud 460800 write-flash 0x10000 build/perf_validation/balance_car.bin > '+EVIDENCE+'/'+label+'_flash.log 2>&1 && sha256sum '+EVIDENCE+'/'+label+'.bin')
elif mode=='collect':
    dest=Path(__file__).parent/'evidence';dest.mkdir(exist_ok=True)
    for ext in ['json','csv','log']:
        run(['scp','develop:'+EVIDENCE+'/*.'+ext,str(dest)])
else: raise ValueError(mode)
