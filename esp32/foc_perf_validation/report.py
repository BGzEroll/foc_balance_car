"""Produce tables from saved evidence, with comparable CSV statistics."""
from pathlib import Path
import csv,json,math
p=Path(__file__).resolve().parent
e=p/'evidence'
summary=json.loads((e/'analysis_summary.json').read_text())
lookup={(x['case'],x['motor']):x for x in summary}
def item(case,motor):return lookup[(case,motor)]
lines=['# 240 MHz / 条件开方 / LUT 实测结果','',
       '日期：2026-10-23。基线为 main `a8d1b4f`；main 未修改。','',
       '## 原 v18 与优化版：相同限幅直接对照','',
       '双方均为 `min(0.4 V, 0.04 × Vbus)`，速度 PI 0.2/0.4、±0.7 A、相同启动斜坡和 4 s 时长。速度为 1.0～3.25 s 机械角度拟合。','',
       '| 指令 rad/s | 电机 | v18 rad/s | 优化版 rad/s | v18 最大 PI 间隔 μs | 优化版最大 PI 间隔 μs |',
       '|---:|---|---:|---:|---:|---:|']
for speed in [10,27]:
    for motor in [1,2]:
        a=item('reference_speed'+str(speed),motor);b=item('matched_lut_speed'+str(speed),motor)
        lines.append(f"| {10 if speed==10 else 27.5} | M{'A' if motor==1 else 'B'} | {a['speed_rad_s']:.4f} | {b['speed_rad_s']:.4f} | {a['max_gap_us']} | {b['max_gap_us']} |")
lines+=['','## 同一台架的逐项对比','',
        '本组均为最新 main 的中心调制与 `min(0.652 V, 0.08 × Vbus)`，外层台架速度环相同，仅变更计算路径。','',
        '| 变体（10 rad/s） | 电机 | 平台 rad/s | 计算路径最大 μs | 采样到提交最大 μs | 最大 PI 间隔 μs | 有效/总采样 |',
        '|---|---|---:|---:|---:|---:|---|']
for case in ['baseline_speed10','cpu240_speed10','sqrt240_speed10','lut_speed10']:
    d=json.loads((e/(case+'.json')).read_text());timing={x['motor']:x for x in d['TIMING']}
    for m in [1,2]:
        a=item(case,m);t=timing[m]
        lines.append(f"| {case} | M{'A' if m==1 else 'B'} | {a['speed_rad_s']:.4f} | {t['compute_max_us']} | {t['command_age_max_us']} | {a['max_gap_us']} | {a['valid']}/{a['active']} |")
lines+=['','计算路径统计从调用 FOC 前到命令提交后，包含目标快照、状态发布和中断/锁等待。最大值是本轮观测值，不能视为硬实时上界；条件开方的独立收益未从有抖动的最大值中稳定分离。','',
        '## 片上数学基准','',
        '在 motor 初始化前、无 PWM 输出时采集。每种路径预热 4 批后计 20×512 次，含相同输入生成、函数调用和求和开销。','',
        '| 路径 | 160 MHz cycles/次 | 240 MHz cycles/次 | 240 MHz μs/次 |',
        '|---|---:|---:|---:|']
a=json.loads((e/'baseline_speed10.json').read_text())['MATH'];b=json.loads((e/'lut_speed10.json').read_text())['MATH']
for x,y in zip(a,b):lines.append(f"| {x['kind']} | {x['cycles_mean']:.3f} | {y['cycles_mean']:.3f} | {y['cycles_mean']/240:.4f} |")
lines+=['','本轮 LUT 数学路径约为同频原路径的 5.3 倍速度；显式 `sincosf()` 没有收益，未采用。不能把这个比例套到整个 FOC。','',
        '## 其他优化版工况','',
        '| 工况 | 电机 | 速度 rad/s | Id 均值 A | 最大 PI 间隔 μs | 有效/总采样 |',
        '|---|---|---:|---:|---:|---|']
for case in ['lut_d030','lut_speed27','lut_negative27']:
    for m in [1,2]:
        a=item(case,m)
        lines.append(f"| {case} | M{'A' if m==1 else 'B'} | {a['speed_rad_s']:.4f} | {a['id_mean']:.6f} | {a['max_gap_us']} | {a['valid']}/{a['active']} |")
lines+=['','## 电流统计的比较范围','',
        '下表只使用双方约 200 Hz CSV 的共同平台窗口；原 v18 的 METRIC 是全速 PI，优化版 METRIC 是约 1 kHz API 快照，不能直接比较。CSV 仍可能受抽取相位混叠，不能据此断言电流纹波或机械震动改善。','',
        '| 指令 rad/s | 电机 | v18 CSV Iq 误差 RMS A | 优化版 CSV Iq 误差 RMS A | v18 CSV Id 误差 RMS A | 优化版 CSV Id 误差 RMS A |',
        '|---:|---|---:|---:|---:|---:|']
for speed in [10,27]:
    for m in [1,2]:
        a=item('reference_speed'+str(speed),m);b=item('matched_lut_speed'+str(speed),m)
        lines.append(f"| {10 if speed==10 else 27.5} | M{'A' if m==1 else 'B'} | {a['sampled_iq_error_rms_A']:.6f} | {b['sampled_iq_error_rms_A']:.6f} | {a['sampled_id_error_rms_A']:.6f} | {b['sampled_id_error_rms_A']:.6f} |")
accuracy=json.loads((e/'math_accuracy.json').read_text())
lines+=['','## 数值检查','',f"真实 helper 的 200 万组主机数值检查通过：LUT 分量绝对误差最大 `{accuracy['lut_component_abs_error']:.4g}`；快速归一化环形角度差最大 `{accuracy['wrap_circular_error_rad']:.4g} rad`；组合替换对 PWM 比较值影响最多 `{accuracy['max_pwm_compare_delta_ticks']}` tick；条件开方输出最大差 `{accuracy['conditional_sqrt_voltage_delta']}` V。",'',
        '包含零点、正负整圈边界及大角度回退路径。数值与名义 PWM 检查不等于示波器对模拟采样窗口的验证。','',
        '结论：速度基本保持 v18 水平，计算余量和本轮最坏更新间隔改善；尚未证明机械震动、绝对电流精度、落地平衡或带载性能更好。']
(p/'RESULTS.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
print('Wrote RESULTS.md')
