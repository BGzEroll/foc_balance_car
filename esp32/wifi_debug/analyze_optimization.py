"""Summarize controlled SDK/transport experiments, including failed runs."""
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
BASE = HERE / "results/optimization"
lines = ["# 历史 UDP / ESP-IDF 优化探索（已撤回，2026-10-23）", "",
         "按用户选择，当前已恢复标准 SDK/socket 路径并限制为 200 Hz。本文件保留历史实验，下文“最终”指该轮实验候选，不代表当前部署。仅修改 codex/test-upper-controller，main 未改变。ESP-IDF 6.0.2，ESP32-S3，112 字节/记录。电机停机，USB 保持连接；不等同于开电机后的实时性验证。", "",
         "## 方法与改动", "",
         "按以下顺序累加修改，每步保留汇总及原始帧。短测为 8～10 秒，统计时去掉开头 1 秒；短测与最终长测分别列出。链路环境没有射频隔离，不能把单次差异当作精确的因果效应。", "",
         "1. pin_core0：只把 lwIP TCP/IP 任务从无核心亲和性改为 core0，隔离 core1 电机任务；单独修改没有提升吞吐。",
         "2. core_lock：开启 CONFIG_LWIP_TCPIP_CORE_LOCKING 和线程安全检查，测量采样、轮询和 sendto 的平均/最大墙钟耗时。",
         "3. buffers：Wi-Fi 动态 TX 缓冲 32→64，TX BA 窗口 6→32，开启 CONFIG_LWIP_IRAM_OPTIMIZATION；提高发送能力但高频时排队仍严重。",
         "4. timer_be / timer_vi：esp_timer 周期通知替代每轮 vTaskDelay(1)；定时回调只唤醒任务，不在回调中执行 socket 操作。TOS 0 / 128 用同一固件对照。",
         "5. cache32 / cache32_vi：S3 指令缓存 16→32 KB （实际构建没有启用 wrap）。这一组合带来明显改善：500 Hz 短测收到约 496 Hz，未观察到序号缺失；原来 sendto 平均约 2.7 ms，缓存优化后该档约 0.42 ms。",
         "6. batch4：每个 UDP 数据报最多合并四条独立时间戳记录（448 字节）；记录频率与数据报频率分别统计。高负载失败保留，不能把记录频率叫作数据报频率。", "",
         "初期只更新应用的各轮没有同步重编的 bootloader；缓存轮次只能视为探索数据。aligned_single 为三份镜像完全匹配后的确认，final_single 的 60 秒测试失败也保留在表中。", "",
         "最终保留：core0 亲和性、core locking/线程安全检查、lwIP IRAM、32KB I-cache（未启用 wrap）、定时任务通知和诊断统计；TX 缓冲恢复 32、TX BA 窗口恢复 6，PHY/带宽恢复自动。本轮未改变看门狗设置，也没有改变 PID、FOC 逻辑或控制周期。", "",
         "## 各轮实测", "",
         "| 版本 | 请求记录 Hz | 每包记录数 | 实收记录 Hz | 序号缺失 % | TCP 指令 RTT P95 ms | 控制最大间隔 ms |",
         "| --- | ---: | ---: | ---: | ---: | ---: | ---: |"]
for name in ["pin_core0", "core_lock", "buffers", "timer_be", "timer_vi", "cache32", "cache32_vi", "batch4", "final_single", "final_batch", "aligned_single", "receiver_linux", "fixed_phy", "retained", "retained_limit", "retained_high", "confirmed_350", "confirmed_final"]:
    path = BASE / name / "summary.json"
    if not path.exists():
        continue
    for r in json.loads(path.read_text(encoding="utf-8")):
        if "error" in r:
            lines.append(f"| {name} | {r['requested_hz']} | {r.get('batch',1)} | 失败 | — | — | — |")
        else:
            lines.append(f"| {name} | {r['requested_hz']} | {r.get('batch',1)} | {r['received_hz']:.2f} | "
                         f"{r['sequence_loss_percent']:.3f} | {r['ping_rtt_p95_ms']:.2f} | {r['control_max_dt_us']/1000:.3f} |")
lines += ["", "## 解释与使用", "",
          "短测 500 Hz 约 496 Hz 的结果不能作为稳定结论；随后 60 秒实收约 273 Hz、序号缺失约 45%。镜像匹配后更高频率仍出现 ENOMEM；Linux 接收端也复现，固定 HT20/MCS0 没有改善，已撤回。", "",
          "每条记录是发送任务读取的快照。现有 IMU/上层控制更新约 200 Hz，因此更高的网络记录率不意味着 IMU 或控制器产生了同样频率的新样本。", "",
          "序号缺失只覆盖接收序号首尾之间。还必须看 NET 的 missed（车端跳过的发送时隙）及 drops（未成功发送的记录）；未见序号丢失不能证明覆盖每个请求时隙。NET 同时报告 sample_us、send_us、poll_us 的平均/最大耗时，errno=12 表示 ENOMEM，heap 为查询时剩余堆。", "",
          "保留单记录模式作为默认，TOS 默认 0；NETTOS 128 是实验选项，不能保证在路由器上得到特定服务质量。超过链路承载能力时，增大缓冲会增加延迟，UDP 不重传过期快照。合包模式会等待凑齐指定记录数，额外延迟上界取决于实际采样间隔；停止订阅时丢弃未凑齐的尾包。", "",
          "命令示例（电机停机）：", "",
          "```powershell",
          "python esp32/wifi_debug/client.py 192.168.114.243 --transports UDP --rates 500 --seconds 60 --tos 0",
          "python esp32/wifi_debug/client.py 192.168.114.243 --transports UDP --rates 1000 --batch 4 --seconds 30 --tos 0",
          "```", "",
          "协议：STREAM UDP 记录Hz 接收端口 每包记录数。记录率支持 0～5000，合包支持 1～8；此范围用于压力实验，并非已验证的可靠工作范围。每包最多 896 字节，保留原 112 字节记录格式。", "",
          "## 最终配置验证", "",
          "retained：每档 30 秒；retained_limit：每档 20 秒；retained_high：每档 30 秒；confirmed_350：60 秒。全部维持单条/包与 TOS=0。超过这些测试时长的长期表现以及开电机后的实时性尚未验证。", "",
          "优化前 UDP 200 Hz 请求实收约 196.17 Hz，指令 RTT P95 28.99 ms，控制最大间隔 8.500 ms；最终 retained 的 200 Hz 实收 200.00 Hz，序号缺失 0%，车端 drops/missed 均为 0，指令 RTT P95 14.40 ms，控制最大间隔 7.001 ms。", "",
          "## SDK 依据", "",
          "- 实际 SDK 源码：components/lwip/lwip/src/api/sockets.c 的 lwip_sendto → netconn_send；components/lwip/Kconfig 的核心亲和性/core locking；components/esp_netif/lwip/netif/wlanif.c 将驱动 ESP_ERR_NO_MEM 映射为 ERR_MEM。",
          "- 官方 examples/wifi/iperf/sdkconfig.defaults.esp32s3 使用 TX 缓冲 64、TX BA 窗口 32、32KB 指令缓存和 wrap。没有照搬 QIO 等涉及硬件假设的选项。",
          "- [Espressif Wi-Fi 性能说明](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-guides/wifi-driver/wifi-performance-and-power-save.html)",
          "- [Espressif lwIP 性能与 ENOMEM 说明](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-guides/lwip.html)", ""]
confirmed = BASE / "confirmed_350/summary.json"
if confirmed.exists():
    r = json.loads(confirmed.read_text(encoding="utf-8"))[0]
    if "error" not in r:
        lines[4:4] = ["## 350 Hz 长测（未通过）", "",
            f"350 Hz 单记录/包，60 秒实收 {r['received_hz']:.2f} Hz，序号缺失 {r['sequence_loss_percent']:.3f}%，"
            f"指令 RTT P95 {r['ping_rtt_p95_ms']:.2f} ms，控制任务最大间隔 {r['control_max_dt_us']/1000:.3f} ms。",
            f"板端统计：`{r['net']}`", "",
            "350/400/500 Hz 持续测试出现拥塞，因此没有将其标为稳定档位。日常使用推荐 200 Hz，较高档仅是本次停机链路测试的结果，不能当作芯片物理极限或开电机后的验证。", ""]
final = BASE / "confirmed_final/summary.json"
if final.exists():
    rows = json.loads(final.read_text(encoding="utf-8"))
    conclusion = ["## 最终 60 秒确认", "",
        "以下为最终保留固件，单条/包、TOS=0；序号缺失与车端 missed 分开列出。", ""]
    for r in rows:
        if "error" in r:
            conclusion.append(f"- {r['requested_hz']} Hz：{r['error']}")
        else:
            conclusion.append(f"- 请求 {r['requested_hz']} Hz，收到 {r['received_hz']:.2f} Hz，序号缺失 {r['sequence_loss_percent']:.3f}%，"
                f"指令 RTT P95 {r['ping_rtt_p95_ms']:.2f} ms，控制最大间隔 {r['control_max_dt_us']/1000:.3f} ms。\n"
                f"  板端：`{r['net']}`")
    conclusion += ["", "这些测试均为停机通信验证；更长时间、干扰环境和开电机后的实时性仍需另行验证。", ""]
    lines[4:4] = conclusion
(HERE / "OPTIMIZATION.md").write_text("\n".join(lines), encoding="utf-8")
print("OPTIMIZATION.md updated")
