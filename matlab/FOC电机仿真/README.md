# ZD2808独立FOC电机仿真

打开 `launch_foc_demo.m`，点击MATLAB编辑器的“运行”。它在当前MATLAB中显示Simulink框图和结果曲线，不启动第二个桌面。修改其中的 `scenario` 可以切换工况：

| 工况 | 用途 | 时长 |
|---|---|---|
| `current` | 锁轴，iq依次为0、2、4、−3、8、0A；8A指令被限制为6A | 0.42s |
| `reverse` | 正转矩、负转矩、零转矩，观察加速、制动与反转 | 0.75s |
| `load` | iq=2A，0.15～0.35s施加0.020N·m负载 | 0.65s |
| `voltage` | iq=6A加速，观察高速时母线电压不足、电流跟踪下降 | 0.8s |

锁轴工况代表电机台架上转子固定，不代表小车站立。带载工况只闭环电流，负载变化会改变加速度，不会自动维持恒定转速。

命令行用法（先在matlab目录运行 `balance_setup`）：

```matlab
run_foc_demo('current');       % 显示框图和波形
run_foc_demo('load',false);    % 后台计算并保存结果
summary=run_foc_suite;         % 四个工况批量验证
p=foc_parameters;             % 查看完整参数和PI系数
```

## 参数来源与换算

修改 `foc_parameters.m`，保存后重新运行工况即可生效。参数不通过外部作者代码加载，不复制或编译作者代码；模型仅使用本工程独立编写的MATLAB函数与Simulink基本模块。需要MATLAB、Simulink和MATLAB Function模块支持，无需Motor Control Blockset或Simscape Electrical专用电机模块。

| 参数 | 默认值 | 来源或假设 |
|---|---|---|
| 极对数 | 7 | 只读核对原作者 `SuperCar.cpp` 的两个 `BLDCMotor(7)` |
| 线间电阻 | 180mΩ | 用户LCR电桥，两根电机线之间测量，第三根悬空；DCR还是交流ESR待确认 |
| 线间电感 | 48μH | 同上；测试频率、转子位置待确认 |
| 等效Rs | 90mΩ | 平衡等效星形电路，线间值除2；未增加温升模型 |
| 等效Ld=Lq | 24μH | 低凸极性初版模型，线间有效电感除2；并非已辨识出d/q电感 |
| KV | 700rpm/V | 用户提供商品页的标称值，未实测 |
| 母线电压 | 7.4V | 用户两节18650串联的工作点假设，可改8.4V；作者代码配置8.2V |
| iq指令限制 | ±6A | 作者配置值，可改；不代表电机或驱动器持续额定电流 |
| 电流采样周期 | 40μs | 此仿真的固定25kHz；作者库PWM默认25kHz不等于其实际电流环频率 |
| 电流环带宽 | 500Hz | 初版设计选择，Kpd=Ld·2πfc，Kpq=Lq·2πfc，Ki=Rs·2πfc |
| 轮组惯量J | 2.996789×10⁻⁵kg·m² | 本项目实测质量分配下的单侧完整轮组惯量；用于独立台架，不是裸电机惯量 |
| 黏性摩擦B | 1×10⁻⁵N·m/(rad/s) | 未辨识的教学假设 |

线间测量值除2后是供dq模型使用的等效星形参数，不据此断言实际绕组一定是星形连接。线间电感随角度、频率和饱和可能变化，后续可以扫描参数误差。

磁链采用SimpleFOC文档的BLDC KV换算约定：

```text
ke_phase_peak = 60 / (2π √3 KV)        [V/(rad/s)]
flux = ke_phase_peak / pole_pairs     [Wb]
Kt = 1.5 pole_pairs flux              [N·m/A_peak]
```

这里dq变换采用幅值不变形式，iq对应相电流峰值；不是母线电流或相电流RMS。算得Kt约0.01181N·m/A，6A对应约0.0709N·m。这个结果依赖KV换算假设，不能视作实测转矩。按此估计，0.10N·m约需8.46A，超过当前6A指令上限。现有3D仿真仍保留独立的理想转矩模型，其±0.10N·m上限没有被这个电机模型验证。

## 框图与求解

`zd2808_foc_demo.slx`中可以看到：工况指令 → dq电流PI、解耦与电压限幅 → SVPWM平均占空比 → PMSM电气及轮组机械方程 → 理想相电流/角度反馈。`Motor_state`的Unit Delay存放当前采样时刻的物理状态，用于固定步长离散积分，不是额外添加一个传感器延迟。

采用正弦PMSM、id目标0、理想传感器、无附加通信延迟。每40μs更新控制，周期内保持alpha/beta电压，电机用4μs子步长RK4积分。逆变器输出为占空比的平均电压，不包含PWM载波、MOSFET开关纹波、死区、采样窗口、齿槽转矩、磁饱和、热效应。母线是固定电压源，允许回馈能量，不验证电池回充或母线过压。模型没有速度外环、弱磁或无感估算。

修改算法函数后运行 `build_foc_demo` 重建SLX（先保存并关闭现有模型），因为MATLAB Function代码嵌入模型；仅修改参数或切换工况无需重建。

## 结果与验证

结果统一保存在 `matlab/results/foc/工况/`：`results.png`曲线、`waveforms.csv`、`simulation.mat`、`parameters.json`、`validation.json`；批量验证另生成 `summary.csv/json`。

检查稳态电流误差、相电流之和、占空比范围、反转与负载响应、电压饱和，以及电气输入能量=铜耗+负载/摩擦功+电磁/机械储能变化。数值通过只代表该参数化模型自洽，尚未验证实际电机。与3D小车的双向耦合是后续阶段，本版没有改动已有3D模型。

原作者参考定位：`BalanceFocCar/lib/SuperCar/SuperCar.cpp`中39～45行是极对数与采样硬件、84～98行是电压和控制模式、128/144行是电流限制；`lib/UserConfig/UserConfig.h`中33～35行是作者电流PI与滤波配置。作者外环为速度模式，仿真接入LQR时应使用转矩/电流指令，不直接照搬其外环。

参考：[商品标称参数](https://rcdrone.top/zh/products/zd2808-v1-9-2808-700kv-brushless-motor)、[SimpleFOC的KV换算](https://docs.simplefoc.com/voltage_torque_control)、[MathWorks PMSM方程与常数定义](https://www.mathworks.com/help/mcb/ref/surfacemountpmsm.html)。
