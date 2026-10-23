# 小车 LQI 固件

当前分支 codex/test-lqi，main保持9faaf7f不变。只读借用 codex/test-upper-controller 的已用硬件/串口框架，再把上层替换为LQI；没有将PID代码合入main，也没有修改原PID工作区。本地已迁回 D:/code/Embedded-related/balance_car；开发机已迁回 /home/bgzerol/embedded_project/foc_balance_car，同样切到 codex/test-lqi，不再使用额外LQI工作树。

## 本次烧录

应用版本 lqi_current_v1-50659c3c；设计标识、镜像哈希、停机状态与数值验证见 DEPLOYMENT.json。完整构建/刷写记录在本地 results/。通过COM10对目标芯片核验MAC后，仅写factory应用并清除OTA启动选择。先备份了旧factory、bootloader、分区表及OTA选择；保留bootloader、分区表、NVS和两份OTA应用槽。应用逐字节回读SHA256与新镜像一致。

上电停机，无线关闭；没有执行CAL、BALANCE、PROBE或任何带电流的启动指令。IMU默认启动初始化后时间戳已非零，STATUS确认running=0、active=0、fault=0。电机尚未校准，也没有完成架空方向或落地LQI验证。

## 控制器

直接使用 MATLAB LQI 的2×8矩阵，200Hz/5ms，状态为路程、速度、俯仰偏差、俯仰角速度、偏航误差、偏航角速度及位置/偏航误差积分。u=-Kaug*error，公共/差分电流混合为左右轮目标，再经原MA=-1、MB=+1映射。每轮目标上限2A；饱和时冻结积分；位置/偏航积分上限0.3m*s、1rad*s。

配置见 lqi_design.json，实际生成的头文件为 ../balance_car/user_lib/lqi_config.h。硬件俯仰配置零位沿用原实机值-0.001564208089rad，未重新辨识；与CAD模型的参考零位分别存储。沿用原坐标观察和编码器方向校准，不以烧录替代实物方向验证。原±2.5A相电流跳闸、1000us采样/指令过期、30ms心跳、传感器新鲜度、倾角/轮速和故障锁存保护均保持。

native_test.cpp与verify_native.py编译真实控制器，对照MATLAB导出矩阵的双精度参考。16,000样本覆盖初始化、里程积分、偏航包角、0.7/2/3A请求上限、饱和冻结与积分限幅；最大输出电流差约1.2e-6A，超过2A的请求仍限制在2A。这是算法验证，不是带电实机闭环证明。

## 串口与记录

921600波特。STATUS查询停机/故障；LQI或GAINS返回LQI设计标识、矩阵与配置；FAULT返回原底层故障。旧SET/TUNE/TUNEA/GAINSSET明确拒绝，不再作为PID改参接口。

UARTSTREAM 200/0仍可订阅/关闭记录。遥测升级为BCU3（224字节，CRC32）；旧BCU2工具不能直接用。原velocity_integral槽为位置误差积分，pitch_integral恒零，yaw_integral为偏航误差积分；旧10个PID增益槽保留为零，完整LQI矩阵由LQI命令获得。BCU3修改只标识上层语义，未提升遥测率。

当前重建：build.py，仅在开发机编译，不拉取bin；数值验证：verify_native.py；开发机USB/IP设备附加且小车停机后，flash.py通过SSH调用开发机device_flash.py完成备份、刷写、回读和停机验证。flash_windows_legacy.py仅保留首次Windows烧录的历史实现，不是现行流程。脚本依赖开发机ESP-IDF6.0.2以及本机既有esp32/wifi_debug/.tools工具库。firmware/与results/含镜像、备份和现场记录，不提交Git。后续若需要带电验证，应先确认车身静止、双轮悬空完成原方向校准，再按受控条件落地；本次烧录没有自动进行这些步骤。

之前出现过的source=2时序故障没有因LQI被宣称修复。本次只验证了刷写、启动、停机通信和算法一致性。

## 迁移与现状

本地及开发机原目录均使用codex/test-lqi；main仍为9faaf7f。2026-10-24迁移仅切换日志进程，没有发送STOP、CAL、BALANCE、刷写或参数修改。原数据、MAT结果、镜像备份和开发机ELF均已保留。当前USB仍由Windows记录，设备3-9已Shared；下次通过USB/IP交给开发机后直接烧录，本轮未切换USB。

迁移后采集曾记录到fault=6的相电流越界保护；没有清故障或自动重启。故障前60秒、12,001样本中，外层目标限流比例约3.0%，上一周期对齐的Iq跟踪RMS约0.0171/0.0221A，采样Iq峰值约2.047/2.061A；这不能排除200Hz未捕获的相电流尖峰或采样异常，也不能据此认定0.08限压导致过流。详情见results/live_20261024_015434/limits_observation.json。
