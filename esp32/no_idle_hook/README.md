# core1专用运行：取消Idle检查与应用hook

本轮按用户要求简化core1调度：不再要求Idle1获得运行时间。应用目前仍为一个合并的sensor_foc任务；没有拆分为两个应用任务。

## 改动

- 取消 `CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1`，只取消IDLE1在任务看门狗中的默认订阅。
- 保留 `CONFIG_ESP_TASK_WDT_EN`、`CONFIG_ESP_TASK_WDT_INIT` 及core0的Idle任务看门狗。
- 保留 `CONFIG_ESP_INT_WDT` 与 `CONFIG_ESP_INT_WDT_CHECK_CPU1`；core1的中断看门狗仍工作。
- 删除应用Idle hook、注册代码和Idle执行次数统计。
- 合并的sensor_foc任务由优先级0改为5，持续运行在core1。循环末尾的taskYIELD仍保留；它不再承担让Idle运行的职责。
- 继续保留“仅新有效电流组更新对应电机PI”、每台电机独立采样dt、core0 ADC同步预触发、原子快照与整组拒绝。
- 运行时通过 `esp_task_wdt_status()` 查询两个Idle任务的实际订阅状态，并观察空闲堆变化。
- 新增 `sdkconfig.defaults` 保存上述配置意图，现有 `sdkconfig` 已同步修改。

这不是关闭所有看门狗，也不是关闭core1调度或中断。SDK的高优先级IPC等系统任务仍可抢占应用任务。

## Idle资源回收核对

安装的ESP-IDF6.0.2经典双核FreeRTOS在 `prvCheckTasksWaitingTermination()` 中遍历共享删除队列，任一核的Idle可取出已不在另一核运行的任务并回收。因此本版本core0的Idle可承担相应回收工作；不必仅为此确保Idle1运行。

若以后增加依赖core1 Idle hook的组件，需要重新审计；当前动态电源管理关闭。固定长期任务的本工程适合把core1作为持续计算核。

## 监控边界

中断看门狗主要检测中断/tick被长期阻塞。普通任务陷入循环、但中断仍正常执行时，不会再靠IDLE1任务看门狗报警。

当前仍为无功率PWM输出的实验固件。本轮没有增加控制进度看门狗、PWM命令超时停机或正式故障状态机；这些在正式电机运行版本中需要单独定义，不能把“中断看门狗保留”当作已经覆盖全部控制卡死。

依据：[ESP-IDF看门狗说明](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/system/wdts.html)，并直接核对安装SDK的任务回收代码与Kconfig。

## 60秒实板结果

固件SHA256：`71d541a4eba69c337947af4c9de167a2d318dbf17f133040485342692e962502`。捕获53份完整JSON观测，最后一帧为启动后59.641秒。

| 指标 | 实测 |
|---|---:|
| core0 Idle任务看门狗状态 | ESP_OK，仍在订阅 |
| core1 Idle任务看门狗状态 | ESP_ERR_NOT_FOUND，已取消订阅 |
| core1中断看门狗配置 | 保持开启 |
| 合并任务优先级 | 5 |
| 稳定段任务循环率 | 8,602次/s |
| M1/M2实际PI更新率 | 8,126 / 8,119次/s |
| 实际PI调用次数与有效更新次数 | 各帧均相符 |
| 传感器 | 稳定段持续就绪、未停更 |
| 稳定段可用堆 | 始终323,780字节 |
| 稳定段历史最小可用堆 | 始终315,372字节 |
| 看门狗、panic、assert、比较器API错误、无效ADC | 均未发现 |
| 估计采样漏周期 | 13 |
| 拒绝采样组 | M1为43、M2为50，均未作为新有效反馈发布 |

这轮验证了无需Idle1检查与应用hook也能持续运行，没有把采样长尾问题宣称为零。与旧版短时测试的吞吐差异不是严格CPU占用对照，后续以正式业务负载下的长时间测试为准。

## 文件与回退

开发机工程：`/home/bgzerol/embedded_project/foc_balance_car/esp32/balance_car`。修改前源码、sdkconfig和固件保存在远端 `build/no_idle_before/`。原目录不存在sdkconfig.defaults；回退时该新增默认配置也应一并撤销或恢复对应选项。

本轮源码、生成器、配置默认值、固件和JSON/串口日志保存在本目录及远端 `build/no_idle_experiment/`。`run_hardware.py` 会烧录并复位，不是只读查看脚本。

原作者参考工程没有修改或编译；代码未自动提交或推送。
