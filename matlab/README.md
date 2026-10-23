# MATLAB工程入口

本分支新增 [LQI仿真入口](LQI仿真/README.md)：打开 `LQI仿真/launch_lqi.m`。本次只运行LQI，包含实测约束、非线性与原生三维验证。

| 目录或文件 | 用途 | 从哪里开始 |
|---|---|---|
| `LQR参数计算/` | 修改机械参数、Q/R、采样周期，计算六状态离散LQR增益 | 打开 `LQR参数计算/calculate_lqr.m`，修改顶部参数后点“运行” |
| `3D仿真/` | 原始3D模型、LQR/PID选择面板、模型构建和报告代码 | 打开 `3D仿真/launch_balance_panel.m`，点“运行” |
| `FOC电机仿真/` | ZD2808电流环、平均逆变器、电机台架及四种工况 | 打开 `FOC电机仿真/launch_foc_demo.m`，点“运行” |
| `results/` | 已生成的曲线、轨迹、仿真数据和验证报告 | 七个工况见 `results/suite/` |
| `.cache/` | 可重新生成的Simulink缓存和代码生成文件 | 一般不用打开 |
| `.matlab_prefs/` | 本机MATLAB配置 | 一般不用打开 |
| `balance_setup.m` | 自动添加本工程路径、固定缓存位置 | 命令行使用前可运行一次 |

计算入口与仿真共用同一套参数：`calculate_lqr.m`。它不依赖Simulink，只需要MATLAB及Control System Toolbox。命令窗口运行 `d=calculate_lqr` 可取得K、A/B、离散模型等；不带输出参数运行时会直接打印K。

更改计算入口中的质量、惯量，是更改控制器设计时使用的参数；3D仿真被控对象的实体参数仍来自原URDF。若要模拟硬件质量或惯量实际变化，还需要同步修改URDF并重建3D模型。仅调整Q、R时，无需重建模型，重新运行工况即可使用新K。

从本目录使用命令行：

```matlab
balance_setup
calculate_lqr
run_balance_case('turn')
run_balance_case('turn',true,[],'pid') % 改用级联PID
run_balance_case('slope_hold',true,6,'pid_current',1,'slope') % PID输出iq，单轮上限6A
```

前进方向约定：USB口一侧为后，远离USB口为前；仿真车体+X向前、+Y向左、+Z向上。

PID增益集中在`3D仿真/pid_balance_parameters.m`，不需先输入Kt才能整定。面板可设置左右“实际/指令转矩增益”检查电机标定误差；实机输出改成电流指令时需要按实际执行尺度重新整定，不能照抄仿真N·m参数。详见`3D仿真/PID控制说明.md`。

新增面板选项“PID：目标电流输出”，先将原增益换算为A制，再以理想电流跟踪和暂估Kt驱动同一3D机械模型。标称Kt约0.011814Nm/A来自700KV假设，并非根据作者PID反推出的实测值；核对细节见[电流接口与作者源码核对](3D仿真/电流接口与作者源码核对.md)。电流模式的入口第三个参数单位为A，默认6A；原LQR/PID转矩模式仍为Nm。

坡道新增“明显坡道”和“坡中停车”工况，默认12°、高约12.8cm。选中后使用“坡道整定”；7～19秒在斜面中段停车，再继续驶出。参数在`3D仿真/balance_terrain_parameters.m`，结果与对比曲线在`results/suite/slope/`。运行`run_slope_comparison`可重做两种控制器的同坡对比；它会如实保留控制失败，不把未倒地当成定位成功。

进一步说明：[LQR参数计算](LQR参数计算/README.md)、[3D仿真](3D仿真/README.md)、[面板操作](3D仿真/演示面板使用说明.md)。
