"""Derive the bounded v18 runner; firmware and scheduling are unchanged."""
from pathlib import Path
r=Path(__file__).resolve().parent
s=(r.parent/'dual_bus/run_dual.py').read_text(encoding='utf-8')
s=s.replace("p.add_argument('--velocity',nargs=3,type=float);", "p.add_argument('--gains',nargs=3,type=float,default=[.060318579,226.194672,0]);p.add_argument('--velocity',nargs=3,type=float);")
s=s.replace("if a.velocity:setting", "setting('GAINS '+' '.join(map(str,a.gains)),'GAINS_OK')\n if a.velocity:setting")
(r/'run.py').write_text(s,encoding='utf-8')
