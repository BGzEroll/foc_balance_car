# 标准 Wi-Fi 调试：TCP 指令 + UDP 200 Hz 遥测

当前实现仅用于 `codex/test-upper-controller`，main 未修改。按用户选择，SDK 性能调优实验已撤回：sdkconfig 恢复原配置，使用官方 Wi-Fi STA 和 BSD socket API、普通 FreeRTOS 网络任务，不使用微秒节拍、core locking 调优、强制 PHY 速率、TOS 调优或合包。

连接 `misakanetwork-2.4g`；STA MAC `34:CD:B0:38:AC:90` 已由用户放行。本次 DHCP 地址为 `192.168.114.243`，变化后以路由器租约或串口日志为准。密码位于被 Git 忽略的 `../balance_car/user_lib/wifi_credentials.h`，示例配置为同目录 `wifi_credentials.example.h`。固件包含凭据，不提交 Git。

## 使用

```powershell
python esp32/wifi_debug/client.py 192.168.114.243 --command STATUS
python esp32/wifi_debug/client.py 192.168.114.243 --command NET
python esp32/wifi_debug/client.py 192.168.114.243 --seconds 30
```

客户端默认测试 UDP 200 Hz。TCP 3333 接收逐行命令并确认；UDP 3334 向同一 TCP 连接来源 IP 发送遥测。客户端先向 UDP 3334 发探测包，再通过 TCP `STREAM UDP 200 接收端口` 订阅。`STREAM UDP 0` 关闭。每个 UDP 数据报一条 112 字节记录，允许 0～200 Hz，超过范围拒绝。保留 TCP 遥测以便对照，日常采用 UDP。

`TUNEA 0.5 0.2 3.756021` 使用等效 B 组电流参数；旧 TUNE 命令拒绝。完整准备流程见 `../tuning_preparation/README.md`；`STATUS`、`STOP`、`CAL`、`BALANCE` 等仍通过统一 console 命令队列执行。CAL 会通电对齐，BALANCE/PROBE 会输出电流，本次通信测试未执行这些命令。网络不提供串口式 DUMP；使用 STREAM 或串口导出。

## 时间与保护

上层控制继续在车上按 5 ms 运行，网络操作均在独立任务中执行。200 Hz 是请求的遥测频率，不保证无线接收端严格每 5 ms 收到一包；帧包含发送时间戳、IMU 时间戳、序号和控制周期编号，处理时按实际时间对齐。电机停机时电流快照可能没有刷新，应检查年龄字段。

TCP 使用 TCP_NODELAY，Wi-Fi 使用官方 WIFI_PS_NONE 关闭省电。网络断开时停止该订阅，车上的平衡控制仍独立执行；保留原传感器、姿态、轮速、电流和 PWM 心跳保护。指令为受信任局域网内的明文协议。每次只支持一个 TCP 调试会话。

## 构建与验证

- `sync_build.py`：校验本机与 develop 均在测试分支、main 基线一致，同步明确文件后编译，并取回匹配的应用、bootloader、分区镜像。
- `flash.py`：检查分支并通过 COM10 刷写；本地保留原 2 MB 固件备份，存放于被忽略的 firmware/。
- `verify_identity.py`：核对两端源码、三份镜像摘要及 SDK 关键选项，确认 main 未变。
- `check_commands.py IP`：要求电机已停机，验证真实指令和输入拒绝；重新应用原 B 组参数并 STOP，不校准或启动电机。
- `serial_log.py --reset`：复位并读取启动日志，仅应在停机时使用。

本次恢复标准路径后的结果保留在 `results/standard_200/`。原始 JSONL/串口日志本地保留但不入 Git。此前基线见 `RESULTS.md`，已撤回的 SDK 压力实验见 `OPTIMIZATION.md`，均不代表当前固件配置。USB 按用户要求保持连接；没有验证拔线或开电机后的运行。
