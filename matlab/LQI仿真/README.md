# LQI 仿真入口

本目录只运行 LQI，不运行 PID/LQR 对照。基于 main@9faaf7f22f017444cb60e80a31227c53518281e4 的测试分支 codex/test-lqi；现已迁回原仓库目录；main引用保持不变。本次不连接、复位、刷写或启动实物。

## 打开与重跑

在 MATLAB 打开 launch_lqi.m 点击运行，可看结果曲线并打开独立三维模型 balance_car_lqi.slx。数据缺失时先运行数值仿真；三维模型尚未生成时运行下列原生三维入口。

```matlab
addpath('matlab/LQI仿真');
run_lqi_suite(false);       % 非线性平面+偏航，含失败停机工况
run_lqi_stress;             % 15°恢复，实际触发2A限流并核对积分冻结
run_lqi_robustness;         % 九组组合不确定性，均用LQI
run_lqi_multibody;          % 原生CAD/轮地接触对象：恢复、脉冲、恒定扰动
plot_lqi_results;           % 数值模型曲线
plot_lqi_multibody;         % 原生三维模型曲线
```

数值部分需要 MATLAB 与 Control System Toolbox；三维部分还需 Simulink、Simscape、Simscape Multibody。网格是 main 对应 LFS 对象的展开文件，内容哈希核对一致；未改几何或CAD质量。第一次三维模型编译需要时间。Windows批处理可将 MATLAB_PREFDIR 指向工程 matlab/.matlab_prefs/lqi，避免用户配置目录权限问题。

## 控制器与单位

状态 x=[路程m，速度m/s，俯仰偏差rad，俯仰角速度rad/s，偏航rad，偏航角速度rad/s]。延续 main 三维对象的前/左/上控制约定；俯仰偏差相对于 main 模型静态平衡角。积分状态为位置误差积分 m·s、偏航误差积分 rad·s，不对全部六个状态积分。

每5ms执行：

- u(k)=-Kx·(x(k)-ref(k))-Ki·xi(k)，单位为公共/差分目标电流A。
- xi(k+1)=xi(k)+0.005·[位置误差；偏航误差]，限幅分别为0.3m·s和1rad·s。
- 左右轮目标为 [(u1-u2)/2；(u1+u2)/2]，每轮独立限幅±2A。
- 输出饱和时冻结积分。物理转矩为 Kt·Iq，不能从动力学删除Kt。

设计用精确零阶保持的Ad/Bd，增广矩阵 Aa=[Ad,0;Ts*C,I]，Ba=[Bd;0]。dlqr对这个八状态系统求解积分最优反馈；这里没有运行无积分控制器作对照。Kaug、Ki、Q/R、闭环极点实部/虚部及机械参数在 results/lqi_suite_summary.json，原始MAT在 lqi_suite.mat。

## 使用的数据与尚未辨识的部分

data_provenance.json 保留源文件绝对路径、SHA256和main基点。本地data目录包含180秒36,001点的有效实测快照、历史参数参考和两份被拒绝模型的证据；为避免提交几十MB原始数据，data及results不入Git。

实测确定：控制及记录周期5ms，每轮目标限流2A，电流对上一周期指令的RMS误差约0.0152/0.0253A，母线约8.058～8.151V。姿态高通残差约0.0785°，角速度高通残差约0.1083rad/s；残差含真实运动，只作为压力测试幅度，不是已分离的传感器噪声或零偏。没有根据平均俯仰值强行校正真实机械平衡角。

CAD质量、惯量、轮径直接来自main机械参数。Kt沿用700KV名义推算约0.011814Nm/A；它不是实测Kt。组合测试使用Kt×0.7/1/1.3、0/5/10ms命令延迟、3ms电流滞后和上述残差幅度。当前内层是理想/一阶电流执行模型，没有完整ADC、FOC开关、电池压降、反电势电压饱和和RTOS调度。实测一周期命令对齐不等于辨识出5ms物理执行延迟。

自由局部辨识模型与CAD约束修正均未通过独立验证，本次没有采用它们的A/B或Kt拟合值。三维接触模型与平面扰动模型分别验证，不能把平面误差接近零当成三维或实物误差。

## 工况与保留失败

数值工况包含5°恢复、0.15N/0.15秒轻推、恒定外力/转矩、0.15m位置与20°转向、观测扰动、弱执行器加延迟，以及故意注入停机。原生三维复用main工况：5°恢复、0.4N/0.15秒脉冲及偏航转矩、恒定0.08N外力与0.0004Nm偏航转矩。

时序保护工况在3秒注入停止驱动后倒下，属于明确保留的失败，并非通过工况。它只演示断力后果，没有模拟出真实1050µs时序故障的根因。不得据此认为LQI修复了底层实时性。

最终数值结果见 results/lqi_suite_summary.json，组合结果见 robustness_summary.json，三维结果见 multibody_summary.json；每个工况同时保存CSV/MAT。所有结论限定于这些仿真假设，没有新的实机闭环验证。
