# ESP32开发入口

已完成[事件等待与FOC/sensor拆分对照](current_loop/event_driven/README.md)：事件拆分、合并事件均实际测试，拆分在本次条件下未提升控制表现；已恢复v18作为默认基线。关于电压限幅、交替采样、ADC2与Wi-Fi、轮组惯量的解释见[问题说明](current_loop/event_driven/问题说明.md)。

最新进展见[双电机、动态母线限幅及车型速度范围](current_loop/dual_bus/README.md)：MA裸转子与MB带轮同时完成小电流闭环及正反27.5rad/s有限时测试。当前采用10kHz PWM、每电机5kHz交替采样，电压矢量上限动态跟随母线；仍是架空台架实验，未接入整车平衡。`balance_car/` 源码已同步本轮开发机实验版本，下面的开环记录是历史阶段。

最新已完成 [PWM 20kHz → 10kHz 实物对照](openloop_sampling/PWM频率对比.md)：保持 MA 采样 10kHz、0.12V/5rad/s，2秒全程窗口有效率由 90.09% 提升至 100%。当前板上为默认停止的 10kHz 实验固件。

已完成 MA 低电压开环实际转动及 MATLAB 采样窗口核查，尚未验证真实电机电流闭环。最新结果见[MA 开环采样实测](openloop_sampling/README.md)：5rad/s 目标对应约 −4.99rad/s 实测速度，全运行速率窗口有效率约 90.09%。

先读[阶段状态与下一步](阶段状态与下一步.md)，其中记录当前架构、最后实测、版本边界及推荐推进顺序。

`balance_car/` 现已包含 ESP-IDF 工程；各实验目录保存对应源码副本、固件与证据。本轮实际构建和烧写在开发机 `test/foc_current` 分支完成，不能把 Windows main 的工程文件当成同一实验版本。各实验目录 README 提供具体入口和回退信息。
