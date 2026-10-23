# Wi-Fi 烧录实现与迁移

OTA 已实现，现使用 ESP-IDF 6.0.2 官方 app_update 接口和 bootloader 回滚。完整操作见 [OTA README](../ota/README.md)。TCP 即时调 PID 仍不需要烧录；OTA 用于后续修改固件。

物理 Flash 为 8 MB，已通过 USB 迁移：NVS/PHY 地址保留，factory 起点 0x10000、大小 1.5 MB，otadata 起点 0x190000、大小 8 KB，ota_0/ota_1 各 2 MB。官方分区工具及迁移脚本均校验布局，原完整 Flash 和单独 NVS 已备份并记录 SHA256。保留 USB 救援及 factory OTA 服务固件。

上传端与固件共同检查 ESP32-S3、balance_car 项目、current_pid_v1 版本族、SHA256、大小和 IDF 镜像校验；授权 token、Wi-Fi 凭据与备份均不进入 Git。HTTP 服务位于端口 3335，仅在受信任局域网使用。

升级先启用电机任务级联锁，等待 STOP 确认，清除上层待运行状态；上传中断不会更改启动分区。完整上传后保持锁定，明确 reboot 才提交启动槽。新固件在初始化前锁定输出，电脑核对硬件健康、镜像身份、TCP 命令、UDP 遥测后确认，60 秒未确认则自动回滚。升级不会自动校准或恢复平衡。

实际验收证据以 [acceptance.json](../ota/results/acceptance.json) 的 complete 与各项状态为准；包括正常更新、传输中断、错误哈希、损坏镜像、维护锁、取消 staged 更新、启动未确认回滚及两个 OTA 槽。没有做运行中刷写或实际拔电实验。

没有启用 eFuse anti-rollback/secure boot，也没有通过 Wi-Fi 修改 bootloader、分区表或 factory。官方依据：[ESP-IDF 6.0.2 OTA](https://docs.espressif.com/projects/esp-idf/en/v6.0.2/esp32s3/api-reference/system/ota.html)。
