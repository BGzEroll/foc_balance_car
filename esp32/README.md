# ESP32开发入口

最新已完成 [PWM 20kHz → 10kHz 实物对照](openloop_sampling/PWM频率对比.md)：保持 MA 采样 10kHz、0.12V/5rad/s，2秒全程窗口有效率由 90.09% 提升至 100%。当前板上为默认停止的 10kHz 实验固件。

已完成 MA 低电压开环实际转动及 MATLAB 采样窗口核查，尚未验证真实电机电流闭环。最新结果见[MA 开环采样实测](openloop_sampling/README.md)：5rad/s 目标对应约 −4.99rad/s 实测速度，全运行速率窗口有效率约 90.09%。

先读[阶段状态与下一步](阶段状态与下一步.md)，其中记录当前架构、最后实测、版本边界及推荐推进顺序。

`balance_car/` 现已包含 ESP-IDF 工程；各实验目录保存对应源码副本、固件与证据。本轮实际构建和烧写在开发机 `test/foc_current` 分支完成，不能把 Windows main 的工程文件当成同一实验版本。各实验目录 README 提供具体入口和回退信息。
