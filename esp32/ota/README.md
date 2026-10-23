# 小车 Wi-Fi OTA

采用 ESP-IDF 6.0.2 官方 `app_update` 双槽更新、bootloader 回滚。TCP 命令 3333、UDP 遥测 3334 保持原接口，OTA HTTP 使用 3335。仅适用于受信任的家庭局域网；上传 token 经 HTTP 发送，没有 TLS/固件签名。SHA256 用于发现传输损坏，不能替代签名认证。

## 日常使用

先结束平衡或校准并确认停机。电脑不需要连接主控 USB；小车保留独立供电、Wi-Fi 在线即可。

```powershell
python esp32/ota/client.py status
python esp32/wifi_debug/sync_build.py
python esp32/ota/client.py update
```

默认 IP 为 `192.168.114.243`，可通过 `--host` 指定新地址。默认镜像为 `esp32/wifi_debug/firmware/balance_car.bin`，可通过 `--image` 指定。只上传应用 bin，不上传 merged、bootloader、分区表。

`update` 会检查停机及单位 schema，上传并核对 SHA256，显式选择下次启动槽，重启后检查镜像 ELF 身份、硬件健康、TCP 命令和 UDP 遥测，再确认新固件。新固件必须在启动后 60 秒内确认，否则自动回滚到之前的有效版本。电脑重启/断网使确认失败时，也保留这个回滚行为，不自动重发升级。

重启会清除 RAM 中的电机校准和临时 PID 参数，回到固件编译的基线并保持 STOP。实测会话/参数文件请先保存，之后按调参流程重新应用、校准和启动。

也可分步执行 `stage`、`status`、`reboot`、`confirm`、`cancel`。`stage` 不修改启动槽，升级锁持续生效；`cancel` 可取消尚未重启的镜像并解锁。启动待确认期间不能 `cancel` 解锁。手工 `confirm` 前必须完成健康/遥测检查，日常优先使用自动完整检查的 `update`。

## 输出保护与校验

- 在 flash 写入前，等待电机任务确认 STOP；升级期间校准/启动 API 拒绝请求，任务还会丢弃已经排队的校准与启动。上层控制器清除待运行状态，命令入口只允许 STOP、STATUS、GAINS、SCHEDRESET。
- 新 OTA 固件从应用初始化前就保持升级锁，确认通过后才允许后续人工校准/启动；确认成功不自动启动电机。
- 逐块写入非运行槽；中断、超时、错误 SHA256、错误芯片/项目/单位版本、镜像校验失败都不改变启动槽。socket 超时 3 秒，总上传上限 120 秒。
- 只有完整验证并收到 `reboot` 才通过 `esp_ota_set_boot_partition` 提交。待确认状态下拒绝再次升级。
- `X-OTA-Token` 必须匹配本地凭据。`token.json` 与生成的 `ota_credentials.h` 已忽略，镜像和整片备份也已忽略，不应公开分发。

## 首次 USB 迁移与恢复

物理 Flash 已核实为 8 MB，布局见 [ota_partitions_8mb.csv](../tuning_preparation/ota_partitions_8mb.csv)：保留 NVS/PHY 地址，factory 1.5 MB，两个 OTA 槽各 2 MB。factory 保留迁移时 OTA 服务固件。后续 OTA 不更改 bootloader、分区表或 factory。

首次迁移使用：

```powershell
python esp32/ota/setup_credentials.py
python esp32/wifi_debug/sync_build.py
python esp32/ota/migrate_usb.py --backup-only
python esp32/ota/migrate_usb.py
```

脚本限制当前测试分支和 main 基线，并核对 MAC `34:CD:B0:38:AC:90`、8 MB 物理容量及分区地址。先保存完整 `firmware/pre_ota_8MB.bin`、单独 NVS 和 SHA256 清单，之后只清 otadata 并写 bootloader/分区表/factory，保留 NVS/PHY。旧 `wifi_debug/flash.py` 会拒绝这套 8 MB OTA 配置，避免按旧 2 MB 流程误刷。

本机凭据请保留。若 token 丢失，可在 USB 下重新生成并迁移；无法靠未知 token 远程替换授权。USB 救援时先确认停机，可再次运行迁移脚本恢复 factory。若要恢复迁移前整片备份，需另外核对备份 SHA256、MAC 和恢复意图后通过 USB 完整写回 8 MB；该操作会撤销当前 OTA 分区布局。

## 验收

`check_hardware.py` 是停机硬件验收脚本，包含真实升级/重启，不用于平衡运行时。它检查未授权、错误 schema、64 KB 后中断、错误 SHA256、损坏镜像、stage 取消、维护锁、正常升级和 60 秒未确认回滚，并覆盖两个 OTA 槽。不会发送 CAL、BALANCE、PROBE。

验收结果见 `results/acceptance.json`，迁移/备份摘要见 `results/usb_migration.json`。这些测试验证停机更新与回滚；不代表已做运行中刷写、实际拔电、全部闪存损坏或所有故障注入。升级时电机始终必须停机。

完整更新后可执行 `python esp32/wifi_debug/verify_identity.py --record-ota`，以设备当前 ELF 身份、构建文件 SHA256 和两端源码核对并刷新运行镜像记录。该命令不会改写固件。

官方机制：[ESP-IDF 6.0.2 ESP32-S3 OTA](https://docs.espressif.com/projects/esp-idf/en/v6.0.2/esp32s3/api-reference/system/ota.html)。未启用 eFuse anti-rollback、secure boot 或任何不可逆烧熔操作。
