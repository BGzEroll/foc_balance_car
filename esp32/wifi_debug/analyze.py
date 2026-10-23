"""Generate a compact, evidence-linked report from actual device measurements."""
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
summary = HERE / "results/final/summary.json"
rows = json.loads(summary.read_text(encoding="utf-8"))
text = ["# Wi-Fi 通信实测（2026-10-23）", "",
        "硬件：ESP32-S3，MAC `34:CD:B0:38:AC:90`，小车 `192.168.114.243`，Windows 有线电脑经路由器连接 2.4 GHz Wi-Fi。每帧 112 字节；每档 15 秒，舍弃开头 1 秒。电机全程停机、未校准。", "",
        "| 协议 | 设置 Hz | 收到 Hz | 序号缺失 % | 接收间隔 P99 ms | TCP 指令 RTT P95 ms | 控制任务最大间隔 ms |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: |"]
for r in rows:
    if "error" in r:
        text.append(f"| {r['transport']} | {r['requested_hz']} | 失败 | — | — | — | — |")
        continue
    text.append(f"| {r['transport']} | {r['requested_hz']} | {r['received_hz']:.1f} | "
                f"{r['sequence_loss_percent']:.3f} | {r['arrival_gap_p99_ms']:.2f} | "
                f"{r['ping_rtt_p95_ms']:.2f} | {r['control_max_dt_us']/1000:.3f} |")
text += ["", "## 使用建议", "",
         "采用 TCP 下发指令并确认执行结果，UDP 发送实时遥测。日常先用 100 Hz：本次两种协议都达到目标，未观察到序号缺失。200 Hz 档实际约 195 Hz，UDP 遥测负载下的 TCP 指令 RTT P95 约 30 ms，低于 TCP 遥测时的约 74 ms；需要更密的观察时可选，但已出现跳过发送时隙。",
         "500/1000 Hz 请求没有带来相应有效采样率，UDP 还出现约 16～17% 的序号缺失；当前实现不宜用于这些设置。TCP 1000 Hz 档还出现 10.665 ms 的控制任务调度间隔，超过现有上层控制器的 10 ms 周期保护阈值。不能将网络饱和测试配置直接用于落地平衡。", "",
         "序号缺失只统计首尾接收序号之间；TCP 的可靠交付不代表每包都准时。表中 RTT 始终是 TCP 指令通道，在相应协议遥测负载下测量，不能误读为 UDP 本身的 RTT。", "",
         "## 证据与范围", "",
         "- 最终扫频：[完整汇总](results/final/summary.json)。逐档 JSON 含板端发送尝试数、失败数、跳过时隙、IMU 新时间戳率和状态。原始 JSONL 本地保留但不入 Git。",
         "- 初版 TCP 发送超时 20 ms，在 200 Hz 测试中曾断连；已改为 200 ms 并重新完整测量。原失败日志保留在本地 `results/sweep.log`。初版 UDP 动态源端口未能到达 Windows；固定 3334 并增加客户端探测后恢复。",
         "- 当前网络实现最多请求 1000 Hz。实测接收率才是本次链路表现，不能把设置值称为达成值，也不能把本次结果称为无线带宽极限。",
         "- 车上控制周期仍为 5 ms；停机调度检查没有覆盖启用平衡控制后的计算与电流环实时性。电流年龄字段在停机时可能很大，这是未刷新快照，不是实时电流。",
         "- 保留本机原固件备份及固件摘要；两端 main 基线仍为 `9faaf7f22f017444cb60e80a31227c53518281e4`。", ""]
extra = HERE / "results/unplugged/summary.json"
if extra.exists():
    text += ["## 拔除主控 USB 后的补测", ""]
    for r in json.loads(extra.read_text(encoding="utf-8")):
        if "error" in r:
            text.append(f"- {r['transport']} {r['requested_hz']} Hz：{r['error']}")
        else:
            text.append(f"- {r['transport']}：请求 {r['requested_hz']} Hz，接收 {r['received_hz']:.2f} Hz，"
                        f"序号缺失 {r['sequence_loss_percent']:.3f}%，"
                        f"指令 RTT P95 {r['ping_rtt_p95_ms']:.2f} ms，"
                        f"电机活动样本 {r['nonzero_active_samples']}，故障样本 {r['nonzero_fault_samples']}。")
    text.append("")
sustained = HERE / "results/sustained/summary.json"
if sustained.exists():
    text += ["## 持续补测（USB 保持连接）", "",
             "按用户要求没有拔 USB；以下数据通过 Wi-Fi 收发。每档 30 秒，仍舍弃开头 1 秒。", ""]
    for r in json.loads(sustained.read_text(encoding="utf-8")):
        if "error" in r:
            text.append(f"- {r['transport']} {r['requested_hz']} Hz：{r['error']}")
        else:
            text.append(f"- {r['transport']} 请求 {r['requested_hz']} Hz：收到 {r['received_hz']:.2f} Hz，"
                        f"序号缺失 {r['sequence_loss_percent']:.3f}%，"
                        f"指令 RTT P95 {r['ping_rtt_p95_ms']:.2f} ms，"
                        f"控制最大间隔 {r['control_max_dt_us']/1000:.3f} ms。")
    text.append("")
(HERE / "RESULTS.md").write_text("\n".join(text), encoding="utf-8")
print("\n".join(text))
