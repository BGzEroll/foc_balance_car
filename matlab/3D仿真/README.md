# PID小车3D仿真

运行`launch_balance_panel.m`或`balance_demo_panel`，均打开当前的PID + 实测FOC面板。当前模型为`实测FOC/balance_car_pid_foc.slx`，控制器仅包含级联PID和左右目标电流输出。

综合场景、参数、能力边界和重现命令见[实测FOC说明](../实测FOC/README.md)。`run_balance_case`作为兼容入口仅接受PID；第3项现在明确为电流档位A。需要精度复核或指定原参数时使用`run_measured_car`。

原生Multibody Explorer计算完成后使用底部时间轴播放；先最大化窗口，随后可用Region Zoom框选车体运动范围。动画重播不触发重新计算。

该目录的共用机械、接触、坐标和报告工具继续用于当前模型。旧LQR/理想执行器比较记录属于历史资料，不是当前控制方案。
