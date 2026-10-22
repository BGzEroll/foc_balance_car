# 最新 main 的 FOC 性能验证

日期：2026-10-23。测试分支：`codex/test-main-foc-perf-240-lut`。
基线 main：`a8d1b4fc4aa608fc89710a3e8f1fd8e1b05571e0`。本地和开发机 main 均未修改。

结论：建议采用 240 MHz、条件开方、快速角度归一化和 4096 点正弦 LUT + 线性插值。与本轮重新烧录的原 v18 相比，常用速度表现基本一致，计算余量和观察到的 PI 最坏更新间隔改善。未证明机械震动、绝对电流精度或落地带载性能优于 v18。

完整数值见 [RESULTS.md](RESULTS.md)。原始日志、CSV、JSON、故障门控和符号记录位于 `evidence/`。

## 最小正式改动

`core_changes.patch` 只修改三个文件，不含台架入口、统计或其他性能重构：

- `sdkconfig`：160 → 240 MHz，保持 O2、固定频率、原看门狗设置。
- `user_lib/hw/motor.cpp`：ADC 超时周期数引用 CPU 配置；CPU 周期换算为 80 MHz PWM tick 使用 `ceil(cycles / 3)`；只有电压平方幅值超限才开方；改用 helper 的归一化和 sin/cos；PWM 未启动时初始化 LUT。
- `user_lib/hw/foc_math.h`：4096 个 float 的单张正弦表，显式放内部 DRAM；余弦用 1024 点偏移，共用同一插值系数，无第二张表。

该补丁基于上述 main 提交生成，在开发机临时目录成功应用，并使用实际 Xtensa 编译器、O2 和 240 MHz 配置对不含台架仪表的 motor 模块交叉编译通过。见 `evidence/production_patch_check.json`。正式补丁尚未应用到 main，也未将整车应用烧录验证。

PWM 仍为 10 kHz，每电机交替采样/电流 PI 仍约 5 kHz；升频并不改变这些频率。PWM 的 8000 tick 周期、160 tick 窗口保护裕量和 ADC 触发时刻保持不变。CPU 频率改变后不能继续使用旧的 `cycles / 2`；本轮已同步修正为配置推导的分频并实测通过采样门控。

没有改变电流 PI、抗积分饱和公式、3 ms 编码器速度滤波、中心 SVPWM 或 main 的 0.08 母线比例。没有增加电流滤波，也没有扩大反馈超时门限。

## LUT 与角度归一化

LUT 占 16384 字节内部 RAM，启动时生成一次，不动态分配。最终 ELF 的表地址为 `0x3fc95fcc`，FOC 入口位于 `0x403800f4` 内部指令 RAM，见 `evidence/final_symbols.txt`。

正常电角度计算来自单圈机械角度，输入幅度较小。快速归一化在 ±64 rad 内用整数圈数扣减，保留负值和边界修正；更大输入继续走原 `fmodf()`，保留通用回退。校准末端的精确 2π 查表通过索引掩码回到零点。

真实 helper 的 200 万组主机数值检查结果：

- sin/cos 最大分量绝对误差约 `5.49e-7`。
- 相对原归一化的环形角度差最大约 `1.43e-6 rad`。
- 两项组合替换时，中心调制 PWM 比较值最多变化 1 tick。
- 条件开方与原式在本批数值检查中输出一致。

主机只用于数值验证，未把 PC 的运行速度当作 ESP32 性能。片上数学基准包含相同输入生成、函数调用和求和开销；240 MHz 时原数学路径约 622 cycles，LUT 路径约 118 cycles，约快 5.3 倍。显式 `sincosf()` 约 512 cycles，反而略慢于快速归一化后的独立 sin/cos（约 496 cycles），因此未采用它。

为了让每个固件都能执行同样的片上数学基准，基线台架也包含评估 LUT。这里的 16 KiB 是相对原 main 的新增 RAM 需求，不能从两个台架镜像的 RAM 差值推算。正式补丁只初始化一次；台架在启动基准和 motor 初始化各初始化一次，以保持正式核心的独立初始化路径。

## 实物对照口径

沿用本会话用户确认的架空、两侧可自由转动、MB 带轮条件；使用 USB/IP 接入同一块 ESP32-S3。母线板载读数约 8.1～8.2 V。每次 ARM 前静止等待 IMU 校准，再自动辨识电机方向/零位。没有落地平衡测试，没有本轮示波器模拟窗口验证，也没有外部电流表标定。

受控顺序为：最新 main 数学路径/160 MHz → 仅 240 MHz → 再加条件开方 → 再加快速归一化/LUT。各变体使用相同的有限时台架速度 PI：0.2/0.4、±0.7 A，前 0.1 s 为零，0.65 s 升速，最后 0.65 s 降速。该台架通过公共 API 约 1 kHz 更新目标；它用于隔离核心计算变化，不等同于对 main 的 `test.cpp` 独立速度任务进行了振动验收。

原 v18 使用存档的精确已测固件 `reference_v18.bin`，本轮重新烧录。v18 左编码器坐标相反，所以使用相反的左方向符号，电角度零偏从本轮自动校准结果迁入；新固件每轮重新校准，零偏会有小幅差异。所有对照速度都转换到校准后的电机正方向，不代表已验收整车前进方向。

最新 main 参数组采用 `min(0.652 V, 0.08×Vbus)`；原 v18 命令最多允许 `min(0.4 V, 0.04×Vbus)`。本轮先保留前一组测试，随后用后一组共同限幅对 v18 和 LUT 版各测 10、27.5 rad/s。最初尝试设置 v18 的 0.08 比例被原固件拒绝，电机未启动，日志保存为 `reference_rejected_limit*`。

优化版还通过双电机 0.30 A、±27.5 rad/s。所有成功轮次无故障并确认停止。最终复位后为 IDLE，`running=0`、`requested=0`、`fault=0`、`aligned=[0,0]`；默认不会校准或输出 PWM。PVCC 供电门控不等于物理断电。

时序计数来自实际 PI 路径和 ISR。计算路径耗时从 FOC 调用前到 PWM 命令提交后，包含快照/中断/锁等待，单轮最大值不是严格上界。v18 METRIC 为全速 PI，优化版 METRIC 为约 1 kHz API 快照，不能直接拿二者 RMS 比较；结果表另列共同平台窗口、约 200 Hz CSV 的统计，但仍存在抽取相位混叠。没有依据这些统计宣称机械震动更小。

## 复现和固件

`remote.py` 对同步、构建、烧录检查本地和开发机分支，拒绝在 main 执行。`variant.py` 支持 baseline、cpu240、sqrt、wrap、sincos、lut；wrap/sincos 的片上数学基准在每个镜像中直接比较，独立电机轮次重点验证 baseline、cpu240、sqrt 和 lut。

```powershell
python esp32/foc_perf_validation/variant.py lut
python esp32/foc_perf_validation/remote.py sync
python esp32/foc_perf_validation/remote.py build
python esp32/foc_perf_validation/remote.py collect
python esp32/foc_perf_validation/analyze.py
python esp32/foc_perf_validation/report.py
```

台架入口包含真实 `motor.cpp`，没有另写一套 FOC。`perf_bench.cpp`、临时电压参数、诊断字段、计数器、手动 ARM 和限时停机仅服务测试，后续不要整支合入车辆 main，迁移应只审阅 `core_changes.patch`。

最终固件 `tested_firmware.bin` SHA256：`371835f885ff994f3a4ae3c4eb7db4896a965e6cea86a4c7e39e40cf10617ab3`。
原 v18 SHA256：`1e18d3630090170f852070ce64e3f7b59fa2f04d27361591e393cffa3cfbc1f6`。

开发机 ELF、各变体固件和烧录日志位于 `/home/bgzerol/embedded_project/foc_balance_car/esp32/balance_car/build/perf_evidence/`。测试前 2 MiB Flash 备份 `before_flash.bin` 的 SHA256 为 `bfb4626ed89a40c2f9d06b1125e966be4194a2499bb9a420250bc2072ac3bced`，覆盖当前引导/NVS/应用使用区域，不是整片 8 MiB 镜像。该备份只保留在忽略的构建目录，未提交。
