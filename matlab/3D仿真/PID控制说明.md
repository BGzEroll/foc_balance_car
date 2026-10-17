# PID控制与整定

打开`launch_balance_panel.m`运行，在控制器下拉框选择PID。也可以在初始化后运行：

```matlab
run_balance_case('recover',true,[],'pid');
run_balance_case('turn',true,[],'pid');
run_balance_suite('pid');
% 两种控制器的七工况均已计算后：
compare_balance_controllers; % 生成同图曲线与IAE、误差、转矩对比CSV
```

所有增益集中在`pid_balance_parameters.m`，保存后重新运行工况即可生效，不必重建模型。PID参数不通过质量、惯量或Kt计算，不调用LQR设计器求K。现有3D被控对象仍需要自己的物理参数。

更在意加减速而非小倾角时，选择“快速（大倾角）”，或运行`run_balance_case('drive',true,[],'pid',1,'agile')`。快速档保持增益不变，前进/后退指令各用1.2秒，允许25°目标俯仰与0.90m/s目标速度。`pid_balance_parameters('agile')`集中设置该档限幅；常规档保持原配置。应分别看实际到达时间、过冲、打滑近似量和执行饱和，不能只通过倾角大小判断快慢。

坡道停车选“坡道整定”：`pid_balance_parameters('slope')`将速度积分增益由0.03提高至0.30，积分状态上限由0.5提高至1.2，目标倾角允许25°，目标速度上限0.30m/s。坡上维持位置需要持续的倾角和驱动力，原来的小积分上限可能不足。它不是单纯把转矩上限调大；默认单轮指令上限仍为0.10N·m。坡道预设需要与原PID平地预设分开比较，不可把增强积分后的收益全部归为PID算法固有优势。

## 控制链

1. 位置P：路程误差转换成速度修正，与轨迹速度指令相加。
2. 速度PI：速度误差转换成目标俯仰角，限制在平衡角附近±8°。
3. 平衡PID：俯仰误差、误差积分和实际俯仰角速度生成左右轮总转矩。微分作用于实际角度，避免目标角突变导致微分冲击。
4. 偏航PID：航向角误差积分、航向角误差和偏航角速度误差生成右减左转矩。
5. 左右分配、单轮指令限幅，再乘以实际执行增益，送入同一个Multibody轮轴。

速度、平衡和偏航积分都有上限；倾角或单轮输出饱和时冻结相应积分。PID中加入积分不等于重新加入LQI，本工程没有LQI控制器。

## 主要参数与单位

| 参数 | 含义 | 单位 |
|---|---|---|
| `position_kp` | 位置误差到速度修正 | 1/s |
| `velocity_kp`、`velocity_ki` | 速度PI到目标倾角 | rad/(m/s)、rad/m |
| `pitch_kp`、`pitch_ki`、`pitch_kd` | 平衡PID到总转矩 | N·m/rad、N·m/(rad·s)、N·m/(rad/s) |
| `yaw_kp`、`yaw_ki`、`yaw_kd` | 航向PID到差转矩 | 同上 |
| `pitch_trim` | 静态平衡角偏置 | rad |
| `speed_limit`、`pitch_target_limit` | 目标速度、目标倾角限幅 | m/s、rad |

当前PID采样周期5ms，运行入口会同步模型采样周期。仿真的执行接口为N·m。实机可以让控制器直接输出iq电流指令，把未知的Kt包含在对象增益中，通过实测整定，不必每次显式做“需求转矩除Kt”；此时增益与限幅的单位、数值都应按A重新配置，不能把N·m数值直接照抄成A。

KV推算Kt和电流环模型仍有帮助，但不能代替电流限制与实际响应检查。PID与LQR都会受到执行增益、方向、带宽、饱和和延迟影响，PID并不自动消除这些影响。

## 模拟未知电机的转矩误差

面板中“实际/指令转矩增益”支持左右分别填写。1表示标称，0.6表示实际只有指令的60%，1.4表示强40%。命令行示例：

```matlab
run_balance_case('recover',true,[],'pid',0.6);
run_balance_case('route',true,[],'pid',[0.8 1.2]);
run_balance_case('push',true,[],'lqr',0.6); % 同样可测试LQR
```

实际转矩=限幅后的指令转矩×执行增益。因此0.10N·m指令限幅、0.6增益时，实际最多0.06N·m。CSV分别记录`tau_left/right_Nm`实际值和`command_left/right_Nm`指令值，PID还记录速度与俯仰目标。该误差模型未加入电气动态、噪声或延迟，独立FOC模型仍未与3D耦合。

## 建议整定顺序

先固定执行增益和输出限制，逐层调整，保留每轮参数与报告：平衡P/D先使倾角稳定，再调速度P、位置P，最后小幅增加积分消除持续偏差；航向单独调。修改后从倾斜恢复、行驶/转向、推扰、地形到持续扰动依次验证，不只看一次能否站住。

AI可以根据峰值倾角、恢复时间、位置误差、饱和占比和不同执行增益的结果辅助整定；应以多工况为准，不能只优化单条轨迹。这里提供的是仿真参数初版，不能直接视作实机已验证参数。

每个工况保存`controller_config.mat/json`参数快照；重新整定前可以先备份该工况目录，避免只留下最后一组参数。标称两组运行完成后，`compare_balance_controllers`生成`results/suite/comparison_lqr_pid.csv/png`，包含同工况的位置/航向积分绝对误差、末端误差、峰值倾角与转矩，不预设哪种控制器更好。

参考：[MathWorks级联控制器整定](https://www.mathworks.com/help/control/pid-controller-design.html)、[PID抗积分饱和](https://www.mathworks.com/help/simulink/slref/anti-windup-control-using-a-pid-controller.html)。
