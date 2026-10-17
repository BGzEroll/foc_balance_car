# 原始3D模型：LQR与级联PID仿真

初学者打开`launch_balance_panel.m`，点击MATLAB编辑器的绿色“运行”，出现中文工况面板。可以选择六状态LQR或级联PID，再选动作并点“运行所选工况，更新3D窗口”。PID参数与整定步骤见`PID控制说明.md`。

平地动作可以在右侧选择“常规”或“快速（大倾角）”。快速档前进30cm的指令从3秒缩短到1.2秒，PID目标速度限幅由0.35提高到0.90m/s、目标俯仰限幅由8°放宽到25°；PID增益相同。快慢也取决于指令轨迹，并不是LQR天然只能慢、PID天然更快。直接运行`demo_balance_agile`会在当前桌面面板中选择PID快速前进/后退并运行。

Multibody Explorer一次显示当前一个工况，不会把所有已计算结果自动轮流播放。计算结束后，在本次弹出的`balance_car_3d_suite`窗口点击动画播放或拖动时间轴。换动作请回面板重新选择并运行。详细步骤见`演示面板使用说明.md`。

入口会复用当前MATLAB中的面板；Windows脚本`open_balance_panel.ps1`检测到已有MATLAB时不会再启动第二个桌面实例。

Simulink缓存与代码生成文件统一写入`matlab/.cache/`，仿真报告写入`matlab/results/`。`.cache`内的文件可删除，下次运行会自动生成；`.m`脚本和`.slx`模型是工程文件，应保留。

命令行运行前，先在本目录运行一次初始化：`addpath('..'); balance_setup`。`launch_balance_panel.m`会自动完成初始化。文中`results/`均指上一级MATLAB目录下的结果目录。

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
run_balance_case('turn',true,[],'pid') % 同一个3D模型，改用PID
run_balance_suite('pid');            % 后台验证七个PID工况
run_balance_case('push',true,[],'pid',0.6) % 实际转矩仅为指令的60%
run_balance_case('drive',true,[],'pid',1,'agile') % 允许大倾角的快速加减速
run_balance_case('slope_hold',true,[],'pid',1,'slope') % 12度斜面中段停12秒，再继续走
run_balance_case('slope',true,[],'lqr',1,'slope')      % 完整通过高坡道
run_slope_comparison;                               % 同坡、同行程比较两种控制器
```

所有运动由Simscape Multibody求解原有三刚体模型，通过原生API打开Simulink和Multibody Explorer。需要MATLAB、Simulink、Simscape、Simscape Multibody及Control System Toolbox。

## 模型与控制器

扩展工程为`balance_car_3d_suite.slx`，运行`build_balance_suite`可从URDF重新生成。重建前先保存并关闭该工程，生成过程会覆盖SLX。原四状态基础工程`balance_car_3d.slx`作为基线保留。

三刚体、两轮关节、完整质心与惯量来自`模型/仿真模型/通用URDF`；控制器设计默认值来自`模型/参数与图片/机械参数.json`，现在集中列在`../LQR参数计算/calculate_lqr.m`中供编辑。外形采用原仿真STL：车身9万面、单轮组4万面；显示简化不改变显式质量和惯量。车身六自由度基座允许横滚、横移和升降。

控制状态按顺序为：

```text
[纵向路程s，纵向速度v，俯仰角偏差，俯仰角速度，偏航角，偏航角速度]
```

纵向路程由轮轴中心实际水平速度投影到车头方向后积分得到；这是路程测量，不是误差积分控制。报告同时记录世界X/Y轨迹。当前控制路程和航向指令，没有加入世界坐标航点导航或侧向位置闭环。

`../LQR参数计算/calculate_lqr.m`集中管理可编辑设计参数，`solve_balance_lqr.m`根据平衡点处的前进/俯仰线性模型和差速偏航模型，精确离散化后求LQR增益。被控对象仍是完整非线性空间多体系统。

```text
[U; V] = -K * (state-reference)
tau_left  = (U-V)/2
tau_right = (U+V)/2
```

U为双轮转矩和，V为右减左转矩差。36项配置由`balance_control_config.m`统一生成，包括控制模式、PID参数、左右执行增益、运动配置和坡道位置指令。嵌入SLX的代码来自`balance_suite_controller.m`，修改后须重建工程；仅修改PID参数或LQR设计参数不必重建，重新运行工况即可。

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

新增` slope `、` slope_hold `两种明显坡道工况：上坡X=0.25～0.85m，坡角12°，升高约127.5mm；平台0.85～1.05m；下坡1.05～1.65m；1.80～1.84m还有左轮经过的3mm凸起。停车目标在上坡中段X=0.55m，7～19秒保持位置，随后继续驶出。可编辑`balance_terrain_parameters.m`更改坡角和尺寸；修改成更陡的数值不代表已通过验证。`configure_balance_terrain.m`按参数更新既有四个地形刚体，不创建新的模型窗口。

面板选择这两个工况时自动选“坡道整定”。PID该预设的目标倾角限幅25°，速度积分增益0.30、积分状态限幅1.2，相比平地预设增强持续负载补偿；LQR仍使用原K，没有坡角前馈或积分补偿。两者使用同样的坡道、轨迹、转矩限制和理想传感器，结果仅代表这两组整定，不是各算法全局最优对决。普通`run_balance_suite`仍只跑原七个工况，新坡道用`run_slope_comparison`。

接触假设：刚度10000N/m、阻尼10N·s/m、静/动摩擦系数0.9/0.8、临界速度0.001m/s、过渡区0.1mm。求解器ode15s，最大步长1ms，相对/绝对容差1e-5/1e-7。轮胎宽度、变形、胎纹和车身倒地碰撞未建模；俯仰超过45°或横滚超过30°提前停止并判失败。

## 输出与验证

`results/suite/工况_lqr/`和`工况_pid/`保存MAT原始数据、控制状态/指令CSV、世界姿态CSV、曲线PNG、平面轨迹PNG及验证JSON。`summary_lqr.csv/json`和`summary_pid.csv/json`分别汇总两种控制器。执行增益不等于1的结果单独保存在`results/suite/robustness/`，避免覆盖标称工况。旧的`summary.csv/json`属于历史LQR报告，新验证请使用带控制器后缀的文件。

快速档写入`results/suite/agile/`，不会覆盖常规档。`compare_balance_motion_profiles`根据两档PID行驶结果生成快慢对比曲线与CSV。报告另记录第一次前进到27cm的时间（相对1秒起动时刻）、近似轮胎滚动滑差、转矩饱和占比。滑差由轮速、车身绕轮轴角速度与前向速度估算，是模型内的近似量，不能当成实车轮胎抓地验证。快速档姿态检查放宽到40°，45°倾倒提前停止保持原样；地形档目前保持原速度。

坡道预设写入`results/suite/slope/`。停车评估使用12～18.5秒的世界X误差、位置变化范围、速度和上坡面支持力；要求整个检查窗口主要由上坡面支撑，位置摆动小于3cm、平均速度小于0.01m/s、峰值速度小于0.03m/s，再单独检查平均位置误差是否在±2cm内。`hold_stability_pass`和`hold_tracking_pass`分别记录“能在坡上停稳”和“停在指定位置”。完整`pass`还要求后续行驶与末端位置/偏航通过，不能只凭没有倒地判定成功。

坡道轮胎滑差改用接触支持力加权切线，避免把初始落地竖直速度误当作打滑；仍是近似诊断量。模型尚未加入车身与地面的碰撞、真实胎形/变形、真实电气执行限制，因此爬坡成功不代表实物底盘间隙、电机温升或电池电压已经验证。

检查完整结束、未倾倒、姿态范围、末端跟踪误差与速度、实际行驶/转向幅度。地形要求真实接触支持力与抬升，推扰要求对应输入脉冲。末端误差取最后0.5秒平均值。持续扰动工况用于观察LQR的稳态偏差，不要求它消除恒定扰动造成的误差。

验证结果只代表所列理想模型、参数与工况，不等于实物性能验证。

LQR/PID使用同一个三刚体、轮地接触和地形模型。仿真验证范围以各工况`validation.json`与对应`summary_lqr/pid.csv`为准，执行增益测试只验证报告中的离散取值，不代表任意电机都能稳定。


