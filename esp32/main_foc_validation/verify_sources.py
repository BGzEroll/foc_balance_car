"""Read-only remote source comparison against the tested local source manifest."""
from pathlib import Path
import json,subprocess,shlex
r=Path(__file__).resolve().parent;d=json.loads((r/'firmware_manifest.json').read_text(encoding='utf-8'))
prefix='/home/bgzerol/embedded_project/foc_balance_car/'
paths=list(d['source_sha256_lf'])
assert all(' ' not in p and all(c.isalnum() or c in '/._-' for c in p) for p in paths)
code=('import hashlib,json;from pathlib import Path;'
      'root=Path('+repr(prefix)+');paths='+repr(paths)+';'
      "print(json.dumps({p:hashlib.sha256((root/p).read_bytes().replace(b'\\r\\n',b'\\n')).hexdigest() for p in paths}))")
text=subprocess.check_output(['ssh','develop','python3 -c '+shlex.quote(code)],encoding='utf-8')
actual=json.loads(text)
assert actual==d['source_sha256_lf'],{p:(d['source_sha256_lf'][p],actual.get(p)) for p in paths if actual.get(p)!=d['source_sha256_lf'][p]}
out={'result':'PASS','files':actual,'note':'Both sides normalize CRLF to LF. sdkconfig semantic settings checked separately.'}
(r/'evidence/source_verification.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
print('PASS: all eight build/source files match the development checkout.')
