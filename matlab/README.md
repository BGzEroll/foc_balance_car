# 原始3D模型：六状态LQR仿真

初学者打开`launch_balance_panel.m`，点击MATLAB编辑器的绿色“运行”，出现中文工况面板。控制器固定为六状态LQR，选动作后点“运行所选工况，更新3D窗口”。

Multibody Explorer一次显示当前一个工况，不会把所有已计算结果自动轮流播放。计算结束后，在本次弹出的`balance_car_3d_suite`窗口点击动画播放或拖动时间轴。换动作请回面板重新选择并运行。详细步骤见`演示面板使用说明.md`。

入口会复用当前MATLAB中的面板；Windows脚本`open_balance_panel.ps1`检测到已有MATLAB时不会再启动第二个桌面实例。

Simulink缓存与代码生成文件统一写入`matlab/.cache/`，仿真报告写入`matlab/results/`。`.cache`内的文件可删除，下次运行会自动生成；`.m`脚本和`.slx`模型是工程文件，应保留。

```matlab
balance_demo_panel                   % 中文选择面板
run_balance_case('turn')              % 原地转向，打开3D窗口
run_balance_case('drive')             % 前进与后退
run_balance_case('route')             % 行驶中转弯
run_balance_case('push')              % 推力和偏航转矩脉冲
run_balance_case('terrain')           % 缓坡、平台、下坡、单侧凸起
run_balance_case('bias')              % 恒定扰动，观察稳态误差
run_balance_case('recover',false)      % 后台计算倾斜恢复
run_balance_case('recover',true,Inf)   % 取消单轮转矩限幅
summary = run_balance_suite;          % 后台验证全部七个LQR工况
```

所有运动由Simscape Multibody求解原有三刚体模型，通过原生API打开Simulink和Multibody Explorer。需要MATLAB、Simulink、Simscape、Simscape Multibody及Control System Toolbox。

## 模型与控制器

扩展工程为`balance_car_3d_suite.slx`，运行`build_balance_suite`可从URDF重新生成。重建前先保存并关闭该工程，生成过程会覆盖SLX。原四状态基础工程`balance_car_3d.slx`作为基线保留。

三刚体、两轮关节、完整质心与惯量来自`模型/仿真模型/通用URDF`；设计读取`模型/参数与图片/机械参数.json`。外形采用原仿真STL：车身9万面、单轮组4万面；显示简化不改变显式质量和惯量。车身六自由度基座允许横滚、横移和升降。

控制状态按顺序为：

```text
[纵向路程s，纵向速度v，俯仰角偏差，俯仰角速度，偏航角，偏航角速度]
```

纵向路程由轮轴中心实际水平速度投影到车头方向后积分得到；这是路程测量，不是误差积分控制。报告同时记录世界X/Y轨迹。当前控制路程和航向指令，没有加入世界坐标航点导航或侧向位置闭环。

`design_balance_suite.m`根据平衡点处的前进/俯仰线性模型和差速偏航模型，精确离散化后求LQR增益。被控对象仍是完整非线性空间多体系统。

```text
[U; V] = -K * (state-reference)
tau_left  = (U-V)/2
tau_right = (U+V)/2
```

U为双轮转矩和，V为右减左转矩差。配置向量为`[工况编号 单轮限幅 平衡角 K(:)']`；不包含控制模式或额外误差积分状态。嵌入SLX的代码来自`balance_suite_controller.m`，修改后须重建工程。

电机保持理想转矩输入，传感器无噪声、无额外传输延迟；采样周期5ms。单轮限幅默认±0.10N·m，可通过入口第三个参数改为任意正数或Inf。它是仿真约束，不代表实物额定性能，改变限幅不改变LQR增益。

## 七个工况

| 名称 | 内容 | 时长 |
|---|---|---:|
| recover | 相对平衡点前倾8°，恢复并回原位 | 6s |
| drive | 前进0.3m，再后退0.2m，最后路程0.1m | 11s |
| turn | 原地转至60°，再回到20° | 9s |
| route | 前进0.6m，途中将航向转至60° | 10s |
| push | 2秒时施加0.4N水平力0.15s，4秒时施加0.002N·m偏航转矩0.15s | 9s |
| terrain | 行进0.75m，通过3mm坡台和左轮0.8mm凸起 | 13s |
| bias | 2秒后持续施加0.08N水平力及0.0004N·m偏航转矩 | 18s |

初始速度均为零；除recover外，从重心偏置对应的平衡姿态出发。指令用五次平滑曲线，同时提供路程和速度、航向和角速度目标；没有强制车体按目标运动。外力施加在轮轴中点，按世界坐标给定。

## 地形和数值设置

平地用Infinite Plane，轮胎中面用半径32.5mm的Disk接触几何。起伏路段由固定在世界坐标的Brick Solid构成，通过独立Spatial Contact Force与两轮接触；平地工况将地形移至地面下1m。

世界X方向0.18～0.30m上坡、0.30～0.40m平台、0.40～0.52m下坡，0.60～0.64m有仅左轮经过的凸起。平台高3mm、单侧凸起高0.8mm，验证范围仅限这些明确尺寸的路段。

接触假设：刚度10000N/m、阻尼10N·s/m、静/动摩擦系数0.9/0.8、临界速度0.001m/s、过渡区0.1mm。求解器ode15s，最大步长1ms，相对/绝对容差1e-5/1e-7。轮胎宽度、变形、胎纹和车身倒地碰撞未建模；俯仰超过45°或横滚超过30°提前停止并判失败。

## 输出与验证

`results/suite/工况_lqr/`保存MAT原始数据、控制状态/指令CSV、世界姿态CSV、曲线PNG、平面轨迹PNG及验证JSON。`summary.csv/json`只汇总七个LQR工况。

检查完整结束、未倾倒、姿态范围、末端跟踪误差与速度、实际行驶/转向幅度。地形要求真实接触支持力与抬升，推扰要求对应输入脉冲。末端误差取最后0.5秒平均值。持续扰动工况用于观察LQR的稳态偏差，不要求它消除恒定扰动造成的误差。

验证结果只代表所列理想模型、参数与工况，不等于实物性能验证。

当前仅含LQR的工程已在本机R2025b重建并复测，七个工况全部通过；汇总见`results/suite/summary.csv`。
