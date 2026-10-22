# main 抽取版 FOC 与 v18 实物对照

日期：2026-10-22。测试分支：`codex/test-main-foc-validation`。

结论：补齐 v18 的编译、链接和运行配置后，抽取版核心能够正常完成双电机电流阶跃、10 rad/s 和 ±27.5 rad/s 测试，实际 PI 更新约每电机 5 kHz，最大更新间隔恢复到约 200～400 μs。未发现必须重写 Clarke/Park、PI 或调制公式才能运行的问题。原 `main` 未修改，仍为 `fcaeb92cf4a849c5c7f4735986596b2ae573d460`。

MB 带轮还完成了 27.5、±60、100 rad/s 的扩大电压范围测试，观察到约 0.30、0.61、1.00 V 的实际电压矢量幅值。100 rad/s 的电流误差明显增大，不能把速度通过当成高转速精密电流控制已经验收。

## 为什么开始表现得像跑不起来

v18 的运行条件并不全在 `foc_current.cpp` 内。最重要的遗漏如下：

| 条件 | v18 | 原 main | 本测试分支 |
|---|---|---|---|
| FOC 文件速度优化 | CMake 对 FOC 单独 `-O2` | 没有迁移 | 已恢复 |
| 整个 FOC 及数学函数内部 RAM 布局 | `foc_current.lf` 的 `noflash` 映射 | 没有迁移 | `main/motor.lf` |
| MCPWM 控制函数 IRAM | `CONFIG_MCPWM_CTRL_FUNC_IN_IRAM=y` | 关闭 | 已恢复 |
| CPU1 持续轮询与 Idle1 监控 | core1、priority 5 合并轮询，不监控 Idle1 | 同样持续轮询，却监控 Idle1 | 与 v18 对齐 |

原 main 的构建列表还引用已删除的 `bat.cpp`，应用入口仍是旧传感器显示测试，未调用 `motor::init()`。这是构建与接入尚未完成，不能据此评价 FOC 数学核心。

原样编译证据见 `original_build.log`。缺少 IRAM 控制函数配置时，曾记录 PWM 提交从周期相位 6972 开始、结束到 471，跨过周期零点并触发保护；`fault_diag2.log` 保存该失败。调试构建的计时探针曾测得传感器处理峰值约 390 μs、FOC 更新峰值约 341 μs。原始 ADC 继续更新，控制命令却超过 1 ms，于是提交保护提前停机。

完整恢复 `-O2` 和 `noflash` 后，同样的原始 1 ms 命令检查可以保留并通过。最终 ELF 中 `foc::update`、`sinf`、`cosf`、`sqrtf` 均位于 `0x403...` 内部指令 RAM，见 `evidence/final_linker_symbols.txt`。

前期曾试验事件等待、任务拆分、分核、计数通知以及命令发布时间计时。这些候选没有作为最终方案；最终恢复原来的 core1 合并轮询、priority 5、原始采样时间戳命令检查。没有通过扩大 1 ms 时限、过滤电流反馈或更改 PI 来使测试通过。中间失败日志和源码快照均保留于 `evidence/`、`experiments/`。

## 最终改动范围

- `main/CMakeLists.txt`：接入测试入口及 MCPWM 依赖；对 FOC 编译单元单独 `-O2`；加载 `motor.lf`。其余模块仍保持原调试优化配置。
- `main/motor.lf`：迁移 v18 的 FOC 与数学函数 noflash 映射。当前台架把真实 `motor.cpp` 包含到 `foc_bench.cpp` 编译单元，故同时列出 `foc_bench` 和正式模块 `motor`。
- `sdkconfig`：恢复 MCPWM 控制函数 IRAM 和与 v18 一致的 Idle1 设置。中断看门狗、Idle0 监控、3 ms FOC 心跳、1 ms 电流反馈/命令检查、1.5 ms 编码器检查和过流保护仍保留。
- `motor.cpp`：修复读取样本/母线/跨核心跳时的时间戳先后顺序及无符号下溢；故障快照补齐 `fault.active`；未启用电机不承担运行反馈新鲜度义务。另有测试用限时停机、PI 计数和电压观测。
- `foc_bench.cpp`：仅测试使用，默认 PWM 停止，显式 ARM 后校准，每轮由 PWM 中断按时结束；通过 `set_target_current()`、`get_info()` 在电流环外实现 1 kHz 台架速度 PI 和记录。速度斜坡、PI 0.2/0.4、±0.7 A 限制与 v18 对齐。

电流 PI 仍是 Kp=`0.060318579`、Ki=`226.194672`，电流反馈无新增滤波，编码器原有 3 ms 速度滤波保留。没有修改整车上层控制器的 200 Hz 默认设置。

## 测试和证据口径

用户本轮确认车架架空、MA/MB 自由转动、供电已接好。MA 与 MB 都单独完成自动方向/零位辨识；MB 带真实轮子。使用开发机 ESP-IDF 6.0.2、160 MHz、10 kHz PWM、每电机 5 kHz 交替采样。母线由板上 ADC 实测约 8.1 V，没有软件调节物理母线电压。

v18 对照使用原已测固件，SHA256 为 `1e18d3630090170f852070ce64e3f7b59fa2f04d27361591e393cffa3cfbc1f6`，本轮重新烧录，同一硬件复测 10 和 27.5 rad/s。旧左编码器曾取反，main 已取消取反；对照时做了方向坐标换算，不能直接照抄旧 direction 数值。每轮新校准零位有差异，速度正负号是校准后的电机坐标，不代表已验收整车前进映射。

最终代表性结果：

| 工况 | 结果 |
|---|---|
| 双电机 0.30 A，3.5 s | 最终平台均值 MA 0.299527 A、MB 0.299523 A；两侧各 17500/17500 采样有效 |
| 双电机 −27.5 rad/s，4 s | MA −27.346、MB −27.517 rad/s；PI 更新 20000/19984 次，最大间隔 215/400 μs |
| 双电机 +27.5 rad/s，4 s | MA 27.375、MB 27.528 rad/s；PI 更新 19998/19986 次，最大间隔均 400 μs |
| MB +60 / −60 rad/s | 约 +60.030 / −60.023 rad/s；观察电压峰值约 0.610 / 0.607 V |
| MB 100 rad/s，两轮 | 约 99.943 / 99.841 rad/s；观察电压峰值约 1.009 / 0.999 V |

完整自动生成结果见 `RESULTS.md` 和 `evidence/analysis_summary.json`。`*_gate.json` 的 PASS 只表示本轮速度、窗口有效率、更新间隔、限幅和停止条件通过，不等于精密电流控制或机械安全认证。

样本有效性计数、PI 更新次数与最大 PI 间隔来自 ISR/实际 PI 路径。最终 `METRIC` 电流统计来自约 1 kHz API 快照，CSV 为约 200 Hz；旧 v18 的 METRIC 是全速 PI 统计，因此不能直接拿两者 RMS 宣称谁更优。分析脚本另外提供相同平台窗口、近似相同 CSV 采样率下的结果，但抽取相位仍可能混叠。电流仍按标称增益换算，未做外部电流表标定。

## 两节电池与电压上限

原 `min(0.12 V, 0.015×Vbus)` 是低幅值实验限制，不是两串电池能够提供的电压上限。PI 应根据电流误差请求电压，动态限幅只约束最大可用值，而不是强行输出上限。

直接增大原居中调制的幅值会挤占固定低侧采样窗口。本分支的可选 WIDE 模式对三相占空比作相同共模平移，保持线电压，把最小比较值固定为 320；限定矢量幅值比例不超过 0.24 时，最坏比较值不超过 1983，低于原中性比较值 2000，因此保留至少原来的名义低侧窗口。仍使用逐样本窗口判断，不改变 ADC 触发时刻。

从真实调制函数提取并编译的 2799360 组组合检查通过，见 `evidence/modulation_check.json`；这只是计算与名义窗口几何检查，不替代示波器确认运放建立、开关噪声、死区和实际低侧导通。

扩大范围的 MB 测试使用 `min(绝对上限, ratio×实测母线)`：27.5/60 rad/s 为 0.8 V / 0.10，100 rad/s 为 1.8 V / 0.22。静态 0.30 A 在上限 0.8 V 时实际只需要约 0.039 V；100 rad/s 实际约 1 V，说明输出随需求变化。普通 4.2 V 满充锂电两串可按最高 8.4 V 规划，但不能把本轮约 1 V 的电机矢量输出测试写成已经验证满母线调制。

100 rad/s 时 API 快照中的 Iq 误差 RMS 约 0.17～0.20 A、Id 误差 RMS 约 0.22～0.24 A，明显高于低速。具体成因尚未隔离，可能涉及顺序 A/B 采样、PWM 电流纹波、角度/输出延迟与 PI 带宽；下一步需要完整 5 kHz 记录及示波器证据，不能只继续放宽电压或认定精简版核心算法错误。没有落地平衡或整车带载验收。

## 复现与恢复

当前 checkout 和开发机均必须处于测试分支。`remote.py` 对写入、构建、烧录做分支检查，拒绝在 main 上执行。

```powershell
python esp32/main_foc_validation/remote.py sync
python esp32/main_foc_validation/remote.py install-tools
python esp32/main_foc_validation/remote.py build
python esp32/main_foc_validation/remote.py flash
python esp32/main_foc_validation/remote.py collect
python esp32/main_foc_validation/analyze.py
```

`flash` 只写应用区，并要求原始备份已存在。测试命令在开发机目录 `/home/bgzerol/embedded_project/foc_balance_car/esp32/balance_car/build/main_foc_evidence` 内执行，例如使用 ESP-IDF 的 Python 运行：

```text
run_new.py repeat --command "SPEED 3 27.5 4000 0.4 0.04"
gate.py repeat
```

`run_new.py` 会先保持静止等待 IMU 校准，ARM、执行有限时命令、确认停止后导出。串口重新打开会导致本板复位；固件默认停止，测试助手会重新校准。离线分析和 gate 不接触串口。

原测试前 2 MiB Flash 备份位于开发机 `build/main_foc_evidence/before_flash.bin`，覆盖当前工程使用的引导、NVS 和应用区域，并非 8 MiB 整片镜像；SHA256 为 `bdf9fb54d46b4395fe9c0104f900d55433b2cc092a20fe16c5a5899475d64274`。它留在忽略的构建目录，未收入本分支。

最终固件为 `tested_firmware.bin`，SHA256 为 `f3f2d9bbf65fc5c79c774df1b35106a065a1c8aaf533f49ac7f1a4d0e2f2a601`；源码校验见 `firmware_manifest.json`。ELF、MAP、烧录日志保留在开发机 `build/main_foc_evidence/final_tested.*`。最终停止状态见 `evidence/final_stopped.log`。

本分支包含台架入口和证据，不应整支直接当作整车应用合入。正式集成时保留 motor 的 `-O2`、`motor.lf`、SDK 配置和必要小修，将应用入口改为自己的车辆逻辑；不需要采用中间的任务拆分方案。
