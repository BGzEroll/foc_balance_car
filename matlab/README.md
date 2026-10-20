# 小车仿真入口：PID + 实测FOC

当前运行入口仅保留PID，控制链为：位置P → 速度PI → 俯仰PID/偏航PID → 左右目标Iq → base电流环 → 电机电气模型 → Multibody小车。观测包含实测IMU有色扰动、残余零偏、编码器量化和延迟。

当前上层PID与编码器观测默认1kHz，IMU及姿态滤波仍200Hz；面板保留原5ms周期对照。电流PI参数、每侧5kHz电流更新与50us电气积分不变。周期验证和新结果位于`实测FOC/pid_rate_validation/`。

打开`实测FOC/launch_measured_car.m`并运行，即可选择综合场景、电流/电压档位、PID参数对照和噪声开关。旧的`3D仿真/launch_balance_panel.m`也转到这个PID面板。

| 目录 | 用途 |
|---|---|
| [实测FOC](实测FOC/README.md) | 当前PID小车模型、参数、综合场景与验证 |
| `实测FOC/pid_validation/` | PID对照、性能边界与电气压力测试 |
| `实测FOC/results/` | 每次原生3D仿真的MAT/CSV/PNG/FIG/JSON |
| `FOC电机仿真/` | 单独的电机教学/台架模型 |
| `3D仿真/` | 共用机械模型构建、接触与坐标工具，以及兼容入口 |
| `LQR参数计算/`、早期SLX与旧结果 | 历史资料，不加入当前控制器菜单或默认运行路径 |

```matlab
balance_setup
measured_pid_panel
run_measured_car('mixed',true,0,true,500e-6,[],'selected','explore6')
run_measured_car('recover',false,3,true,250e-6,[],'selected','bench')
```

性能探索档为纯仿真：1.5/3/6/10A指令上限、0.45倍母线电压矢量上限；原台架对照档为0.7A及原低电压限幅。高电流、高速与地形能力是模型外推，未作为实物额定值验证。默认base电流PI保持用户已确认数值。

方向：X右、Y前（USB侧）、Z上；MA左、MB右；正轮速前进、正偏航左转。当前入口不再运行LQR或理想目标电流直接转矩执行器。
