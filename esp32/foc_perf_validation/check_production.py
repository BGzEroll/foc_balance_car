"""Check the production-only patch in a temporary tree, then cross-compile motor."""
from pathlib import Path
import subprocess, tempfile, json, shlex, hashlib
p=Path(__file__).resolve().parent
root=next(x for x in p.parents if (x/'.git').exists())
assert subprocess.check_output(['git','branch','--show-current'],cwd=root,text=True).strip()=='codex/test-main-foc-perf-240-lut'
manifest=json.loads((p/'production_manifest.json').read_text())
assert subprocess.check_output(['git','rev-parse','main'],cwd=root,text=True).strip()==manifest['base_main']
stage=Path(tempfile.mkdtemp(prefix='foc_production_patch_'))
for file in ['esp32/balance_car/user_lib/hw/motor.cpp','esp32/balance_car/user_lib/hw/motor.h','esp32/balance_car/user_lib/hw/sensor.h','esp32/balance_car/sdkconfig']:
    q=stage/file;q.parent.mkdir(parents=True,exist_ok=True)
    q.write_bytes(subprocess.check_output(['git','show','main:'+file],cwd=root))
subprocess.run(['git','apply','--check',str(p/'core_changes.patch')],cwd=stage,check=True)
subprocess.run(['git','apply',str(p/'core_changes.patch')],cwd=stage,check=True)
for file,expected in manifest['production_source_sha256_lf'].items():
    actual=hashlib.sha256((stage/file).read_bytes().replace(b'\r\n',b'\n')).hexdigest()
    assert actual==expected,(file,actual,expected)
commands=json.loads((root/'esp32/balance_car/build/perf_validation/compile_commands.json').read_text())
entry=next(x for x in commands if x['file'].endswith('/user_lib/perf_bench.cpp'))
original=shlex.split(entry['command']);args=[];skip=False
for arg in original:
    if skip:skip=False;continue
    if arg in ['-o','-MF','-MT']:skip=True;continue
    if arg in ['-MD','-c',entry['file']]:continue
    args.append(arg)
args+=['-fsyntax-only',str(stage/'esp32/balance_car/user_lib/hw/motor.cpp')]
subprocess.run(args,cwd=stage,check=True)
result={'result':'PASS_PATCH_AND_TARGET_COMPILE','base_main':manifest['base_main'],'temporary_tree':str(stage),'files_verified':list(manifest['production_source_sha256_lf']),'note':'Uses actual Xtensa -O2 compiler and 240 MHz generated config. No bench code in production motor.'}
(p/'production_patch_check.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result))
