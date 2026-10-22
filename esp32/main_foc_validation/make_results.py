"""Generate report tables from saved evidence only; never starts hardware."""
from pathlib import Path
import json,subprocess,sys
r=Path(__file__).resolve().parent
subprocess.run([sys.executable,str(r/'analyze.py')],check=True,stdout=subprocess.DEVNULL)
rows=json.loads((r/'evidence/analysis_summary.json').read_text(encoding='utf-8'))
index={(x['case'],x['motor']):x for x in rows}
required=['final_speed10','final_positive','final_negative','final_d030','final_mb100']
assert all((x,2) in index for x in required),'Final evidence not collected yet'
def f(x,n=4):return '—' if x is None else f'{x:.{n}f}'
out=['# 实测结果表','',
'由 `make_results.py` 从已保存 JSON/CSV 生成。速度采用共同的 1.0 s 至结束前 0.75 s 平台窗口对机械角度拟合。',
'','## 同硬件旧 v18 与最终抽取版','',
'| 指令 rad/s | 电机 | v18 平台 rad/s | 抽取版平台 rad/s | v18 最大 PI 间隔 μs | 抽取版最大 PI 间隔 μs |',
'|---:|---|---:|---:|---:|---:|']
for target,old,new in [(10,'reference_speed10','final_speed10'),(27.5,'reference_speed27p5','final_positive')]:
 for m in [1,2]:
  a=index[old,m];b=index[new,m]
  out.append(f'| {target} | {"MA" if m==1 else "MB"} | {f(a["speed_rad_s"])} | {f(b["speed_rad_s"])} | {a["max_gap_us"]} | {b["max_gap_us"]} |')
out+=['','## 最终与扩大电压范围轮次','',
'| 轮次 | 电机 | 平台 rad/s | Id 均值 A | Iq 误差 RMS A（API 快照） | 观察电压峰值 V | 最大 PI 间隔 μs | 有效/总采样 | 实际 PI 更新次数 |',
'|---|---|---:|---:|---:|---:|---:|---:|---:|']
names=required+['wide_d030_mb','wide_mb27p5','wide_mb60','wide_mb_negative60','wide_mb100']
for name in names:
 d=json.loads((r/'evidence'/f'{name}.json').read_text(encoding='utf-8'));st=d['STATUS'][-1]
 for m in [1,2]:
  x=index.get((name,m))
  if not x or not x['active']:continue
  updates=st.get('pi_updates',[None,None])[m-1]
  out.append(f'| {name} | {"MA" if m==1 else "MB"} | {f(x["speed_rad_s"])} | {f(x["id_mean"],6)} | {f(x["iq_error_rms_A"],6)} | {f(x["voltage_max_V"],6)} | {x["max_gap_us"]} | {x["valid"]}/{x["active"]} | {updates if updates is not None else "未单列"} |')
out+=['','最终 METRIC 来自约 1 kHz API 快照，v18 METRIC 来自全速 PI，二者 RMS 不宜直接比较优劣。`analysis_summary.json` 另列共同 CSV 窗口的抽取统计；它仍可能混叠。',
'','100 rad/s 速度与软件窗口/时序门控通过，但 Id/Iq 快照误差明显增大，未完成高转速精密电流验收。',
'','所有最终轮次均 `fault=0`，结束后 `running=0`、`requested=0`。最后复位默认停止的原始记录见 `evidence/final_stopped.log`。','']
(r/'RESULTS.md').write_text('\n'.join(out),encoding='utf-8')
print('Wrote',r/'RESULTS.md')
