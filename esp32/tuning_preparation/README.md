# 今晚无线 PID 调参与实物—仿真闭环准备

测试分支 `codex/test-upper-controller`；main 不动。PID 仍为 5 ms / 200 Hz，内层 FOC、方向映射、每轮目标电流最大 2 A 和保护逻辑保持。准备阶段不校准、不使能电机。USB 当前仍连接，今晚拔掉的是主控数据线，保留独立供电；拔线后的行为需要当晚确认。

## 已准备

1. TCP 指令 / UDP 目标 200 Hz 的持续会话，无串口操作。`session.py` 同时记录 CSV、参数、命令确认和人工阶段标记；退出仅关闭订阅，不主动 STOP，不重新校准或自动重新启动。
2. 直接电流单位控制律，运行代码不再用 Kt 做输出换算。俯仰与偏航输出合成后直接成为左右 FOC 的目标 Iq。B 组做了等效单位迁移，尚未重新整定。
3. BCW2 每包 220 字节，包括同一控制周期的观测、目标、输出、限幅、实际限流、故障/校准状态、参数版本及完整参数和三路积分状态。保留 BCW1 便于原通信测试。
4. MATLAB 同算法控制器、真实时间戳导入、上一周期指令对齐、局部模型辨识及离线参数搜索。只生成待审核文件，不自动发给小车。
5. 已实现 Wi-Fi OTA、8 MB 双槽布局、上传授权、镜像校验、停机联锁和启动确认/回滚。使用方式见 [OTA README](../ota/README.md)，迁移与验收摘要见 OTA_PLAN.md。

## 新旧参数与单位

旧运行链路把俯仰/偏航 PID 的等效力矩除以 `Kt=0.0133119595 Nm/A`。现在这一步被折入参数，控制律内部没有 Kt。速度 PI 输出目标俯仰、位置 P 输出目标速度，它们不需要除以 Kt。

| 参数 | 昨晚 B（旧尺度） | 等效电流尺度 |
| --- | ---: | ---: |
| 速度 P / I | 0.5 / 0.2 | 0.5 / 0.2 |
| 俯仰 P | 0.4 | 30.0481683 A/rad |
| 俯仰 I | 0.005 | 0.3756021 A/(rad·s) |
| 俯仰 D | 0.05 | 3.7560210 A·s/rad |
| 偏航 P / I / D | 0.105 / 0.01 / 0.02 | 7.8876442 / 0.7512042 / 1.5024084 |

这些增益作用于两轮合计的公共电流或差分电流，仍按 `(common ∓ differential)/2` 混合为每轮指令。旧 TUNE 命令明确拒绝，防止把旧数值误当成新单位。旧有线 B 只是对照起点：用户观察其不倒但来回摆动，不能标为无 USB 时已稳定。

物理模型中的电机力矩常数仍是物理属性；去掉的是实物控制器的在线换算，不能因此从电机/机械动力学里删除电流—力矩关系。新局部辨识模型直接使用实测 Iq 作为输入。

## 今晚的会话

在仓库根目录执行；初始不自动启动电机：

```powershell
python esp32/tuning_preparation/session.py 192.168.114.243 --seconds 7200 --interactive --usb no
```

同一会话中可输入：

```text
STATUS
GAINS
MARK wheels_free_still
CAL
MARK on_ground_held
BALANCE 0 2.0
MARK hands_free_baseline
SET velocity_kp 0.5
TUNEA 0.5 0.2 3.756021
MARK segment_end
QUIT
```

CAL 会通电对齐，只能在用户确认车身静止、双轮自由悬空后执行；需确认 calibrated=1 且两轮方向正确。BALANCE 必须在用户已放稳/拿稳、姿态符合启动范围时执行。`BALANCE 0 2.0` 是连续运行，不因记录窗口结束而断力。上述不是自动脚本，准备阶段没有执行 CAL/BALANCE。STOP 是明确的停机命令，落地时应先拿稳。

`SET key value` 修改一个参数；`TUNEA velocity_kp velocity_ki pitch_kd` 修改三项；`GAINSSET` 一次原子应用全部十项。成功返回 GAINS、单位 schema 和递增 revision；每个控制快照带实际用于计算的那版参数。

需要原子应用审核后的配置，在交互会话输入：

```text
APPLY esp32/tuning_preparation/profiles/wired_B_current.json
```

这里的 profile 是等效旧基线，绝不是推荐新整定值。配置 `schema` 必须为 `current_pid_v1`；未知 schema、NaN、越界、超长报文会被拒绝。没有断线后自动重发启动命令的逻辑。

## 数据与 MATLAB

会话目录包含 samples.csv、metadata.json、events.jsonl。sample_us 是控制取样时间，sent_us 是网络发送时间；cycle 可识别重复与跳过周期。运行时 Iq 测量先于同一行新指令的提交，分析时按 cycle-1 查找上一目标；不能直接拿同一行相减评价跟踪。mode=2 是平衡；flags 的 1/2/4 位分别表示校准、运行、ready。停机电流快照可能旧，必须检查年龄。

```matlab
addpath('D:/code/Embedded-related/balance_car/matlab/wifi_tuning');
folder='D:/code/Embedded-related/balance_car/esp32/tuning_preparation/sessions/本次目录';
[T,metrics]=analyze_session(folder);           % 批处理数值指标
% analyze_session(folder,true);               % 可选绘图，图形环境正常时使用
replay_current_pid(folder);                   % 连续周期上的控制器一步回放
model=fit_current_plant(folder,[5 15]);        % 先人工确认这段落地、松手、无电线牵拉
explore_current_pid(fullfile(folder,'identified_plant.mat'));
```

今晚按此循环：无线无改参基线 → 选固定参数且松手的有效窗口 → 检查缺帧、姿态、限幅、电流跟踪 → 用实测电流辨识局部模型 → 用后 30% 时间段验证 → 仿真筛选候选 → 审核候选/限流/预测指标 → 仅应用一组并重新采集 → 用新数据再修正模型。保留退化结果，不能只挑好看的区间。

辨识拒绝停机/未校准、少于 600 个有效控制周期、多参数版本、长缺帧和激励不足/病态数据。验证误差过大时不进入参数搜索。模型状态为俯仰、俯仰角速度、速度，输入为实测前进合电流 `-IqA+IqB`；限流来自实际记录。它是低频局部经验模型，未包含完整轮地接触、滑移、侧倾和高频 FOC，不等同于完整 Multibody 模型。闭环数据还可能有辨识偏差；仿真候选仅供筛选，不能直接称为实机安全参数。

现有 3D/Multibody 模型与 MATLAB 原参考源保留，未为了匹配今晚数据而随意改质量、惯量或 Kt。待有合格无线落地数据后，先验证局部模型，再决定应修正哪些物理参数和传感器/执行延迟。

## 当前验证证据

- C++ 新旧单位迁移：36,000 样本，最大电流/状态差约 4.77e-7；results/equivalence.json。
- MATLAB/C++ 同算法：36,000 样本，最大差约 6.11e-6；results/matlab_equivalence.json。
- 实际停机 Wi-Fi 首轮采集 2,981 条 BCW2；CSV 可导入 MATLAB，不能用其辨识平衡动力学。
- 合成数据工作流及“拒绝停机数据辨识”测试通过；合成模型不是实物辨识结果。离线搜索若所有候选失败，输出 no_feasible_candidate，旧候选不会被误用。
- MATLAB 离屏绘图在当前批处理环境阻塞，已将绘图设为显式可选；数值导入、辨识、搜索无需绘图即可运行。
- 固件与本地/开发机源码及三份刷入镜像使用 verify_identity.py 核对；不更改 main。

## 2026-10-23 现场更新

用户要求上层每轮目标电流上限改为 2 A，默认 limits 与命令范围已同步。内层目标电流矢量上限原本就是 2 A。现场用户另行明确授权仅将 ADC 相电流跳闸线从 ±2 A 提高到 ±2.5 A；故障锁存、采样窗口判定、ADC 读错及全部超时保护保持原样，没有新增清故障或恢复对齐接口。旧 wired_B_current.json 的 0.7 A 作为历史基线保留；MATLAB 辨识/仿真从实测 current_limit_A 读取本次限流。

新增 FAULT 命令：source=1 有效采样间隔超时；2 PWM 指令样本过期；3 FOC 消费样本过期；4 FOC 更新拒绝，age_or_dt_us 为相应年龄或间隔。未放宽任何超时阈值。

## 无线并行实时性修复

实测原配置 Wi-Fi 关闭时可完成校准；无线数据并行时出现 source=2 的 1051us 指令年龄，ADC 样本年龄约 51us，FOC 心跳约 876us，任务停顿既出现在 sensor::poll 也出现在 taskYIELD 后。不能仅凭停顿位置判定 I2C 驱动阻塞。lwIP 原为无核心绑定、高于 FOC 优先级；改为 CONFIG_LWIP_TCPIP_TASK_AFFINITY_CPU0 后，在 Wi-Fi+200Hz UDP 全负载下双轮校准通过。I2C 双句柄、任务拆分、总线忙延后提交和降低射频功率的试验均已撤回，原 1000us 保护不变。构建现使用 develop 的独立 foc_balance_wifi_test 工作树，原 main 工作区保留。

后续落地复测仍出现过 source=2 超时，因此绑核仅是已验证的阶段性改善，不能称为实时性问题完全解决。上层现在先读取电机信息，再读取传感器包；若编码器快照超出原 1500us 阈值，重读一次再检查，阈值不变。

## 现场 PID 对比

原 B 参数并非已调好的无线参数。live_07 在 2 A 目标上限、±2.5 A 相电流跳闸线下运行约 5.14 秒后 fault=6，故障前指令频繁在 ±2 A 换向。仅将 pitch_kd 从 3.756021 改为 1.878011，live_08 第一段运行约 4.69 秒，末约 400 个有效样本俯仰 RMS 约 0.50°，饱和比例为零，随后上层 result=4、底层 fault=0 停机；该短段不能证明长期稳定。后续重启一段又出现底层 fault=7，需与 PID 震颤分开定位。D 减半档案为 profiles/wifi_D_half_2A.json，输出上限仍为 2 A。

MATLAB 实测单步回放与固件指令最大误差小于 3e-7 A。冻结 live_07 观测比较 D 减半时，523 对连续周期上的饱和比例从约 70.6% 降到 49.3%；这是指令敏感性检查，不是闭环仿真。live_05 排除启动、断帧后最长合格窗口只有 381 点，未通过 600 点辨识门槛，未据此宣称模型有效或发布闭环仿真最优参数。
