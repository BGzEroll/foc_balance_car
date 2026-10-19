# 临时持续双电机观看模式

2026-10-19，用户明确要求实物电机持续转动，直至叫停。沿用先前确认的架空、MA无轮、MB带轮且可自由转动条件。此目录属于电流环实验分支，不改变模型工作区或main。

当前临时固件为v19 WATCH，基于用户敲定的v18合并轮询方案，仅加入持续观看命令和主机租约。d/q PI仍为Kp=0.060318579、Ki=226.194672、Tf=0、D=0。原来的SPEED限时测试仍保留。

- 两侧目标10rad/s，约95.5rpm；速度PI为0.2/0.4，Iq指令上限0.7A。
- 电压矢量上限min(0.40V,0.04×实时母线)；没有采用仿真的高电流/高电压探索档。
- `WATCH 10 2000 25`只允许最多10rad/s和固定2秒租约，正常软启动后保持速度，不周期性停车。
- 主机必须发送`KEEP`续期，超过2秒未续期，MCU在PWM中断内停机；已超时或已有故障时，KEEP不能重新启动。
- 采样/编码器过期、过流、超速、母线和看门狗等原保护保留；后台主机发现异常也发送STOP并退出，没有自动重启。

`watch.py`启动前确认停止、传感器就绪及配置应答。先以3rad/s验证不发KEEP会超时停机、之后KEEP不能重新使能，再明确启动10rad/s持续演示。该实际保护检查已通过，记录见`startup_evidence.log`。`live_snapshot.json`只是取回时刻的运行快照，不应当作始终在线的状态。

开发机运行目录：

```text
/home/bgzerol/embedded_project/foc_balance_car/esp32/current_loop/continuous_watch
```

实时状态在该目录`live.json`，进程号在`watch.pid`，串口原文在`serial.log`，主机状态在`watch_runner.log`。停止时在开发机执行：

```sh
touch /home/bgzerol/embedded_project/foc_balance_car/esp32/current_loop/continuous_watch/STOP
```

主机随后发STOP并把确认结果写入`final_stopped.json`。SIGTERM同样走停止清理；若进程异常退出，MCU租约在2秒内到期。不要在后台进程占用串口时再打开其他串口工具。再次启动需要明确移走STOP文件，程序不自行清除停机请求。

原v18源码和镜像已保存在开发机同目录`baseline.cpp`及`baseline.bin`；对应SHA256：

```text
baseline.cpp a3ca809fa2a8ea6f3fe81d160bbd2c6148a42a39b1d5779851518185498d8c32
baseline.bin 1e18d3630090170f852070ce64e3f7b59fa2f04d27361591e393cffa3cfbc1f6
```

停止后可恢复源码，并把原镜像写回0x10000以回到v18。当前运行中的镜像是v19，不能把它记成已恢复v18。编译使用既有Ninja缓存及工具链，烧录日志已确认写入哈希；未安装依赖或修改开发机环境。

## 手动阻转后的一次明确重启

用户较大力捏住MA后，主机按约1.4秒角度差计算的速度由约9.8降至7.49、随后6.02rad/s，超过原有±3rad/s跟踪偏差门槛，主机发STOP并退出。MCU最后reason=7为请求撤销，没有记录MCU过流停机。原始日志见`squeeze_stop.log`、`squeeze_stopped.json`。用户明确要求重新启动后，重新执行同一脚本及租约检查，仍保持10rad/s、0.7A、base PI及全部保护；未放宽转速偏差门槛。
