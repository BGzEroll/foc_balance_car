"""Compare legacy torque-scaled firmware and direct-current controller on native CPU."""
import base64,json,subprocess
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
files={p.name:base64.b64encode(p.read_bytes()).decode() for p in
       [HERE/'equivalence.cpp',HERE/'reference/old_balance.cpp',HERE/'reference/old_balance.h',
        ROOT/'esp32/balance_car/user_lib/balance.cpp',ROOT/'esp32/balance_car/user_lib/balance.h']}
program='''
import base64,json,pathlib,subprocess,sys,tempfile
with tempfile.TemporaryDirectory(prefix='current_pid_') as folder:
 p=pathlib.Path(folder)
 for n,d in json.load(sys.stdin).items():(p/n).write_bytes(base64.b64decode(d))
 subprocess.run(['g++','-O2','-std=c++17','-Wall','-Wextra','-Werror','equivalence.cpp','old_balance.cpp','balance.cpp','-o','verify'],cwd=p,check=True)
 r=subprocess.run([str(p/'verify')],capture_output=True,check=True)
 print(json.dumps(dict(vectors=r.stdout.decode(),summary=json.loads(r.stderr))))
'''
encoded=base64.b64encode(program.encode()).decode()
r=subprocess.run(['ssh','develop',"python3 -c 'import base64;exec(base64.b64decode(\""+encoded+"\"))'"],input=json.dumps(files).encode(),capture_output=True)
if r.returncode:
    print(r.stderr.decode(errors='replace'));raise SystemExit(r.returncode)
data=json.loads(r.stdout)
(HERE/'results/equivalence_vectors.csv').write_text(data['vectors'],encoding='utf-8')
(HERE/'results/equivalence.json').write_text(json.dumps(data['summary'],indent=2),encoding='utf-8')
print(data['summary'])
