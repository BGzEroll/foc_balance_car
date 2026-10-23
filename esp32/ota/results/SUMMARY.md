# OTA 实物验收（2026-10-23）

测试分支 codex/test-upper-controller，main 未修改。全过程停机，未发送 CAL、BALANCE、PROBE；USB 仍插着，OTA 传输/重启/确认走 Wi-Fi。

- 完整 8 MB Flash 与 NVS 备份后，通过 USB 迁移为 factory + 两个 2 MB OTA 槽；保留 NVS/PHY。
- 未授权 401、错误 schema 400、64 KB 后中断恢复、错误 SHA256 422、匹配传输哈希但镜像损坏 422 均通过；失败未改变启动槽。
- 正常 Wi-Fi 升级、镜像身份校验、TCP/UDP 健康确认、stage 取消与维护锁通过。
- 新固件未确认保持输出锁，60 秒后实测 ota_0 回滚至 ota_1。
- 两个 OTA 槽均已实测正常启动确认。最终固件 917488 字节，上传实测 10.13 / 10.44 秒（不含重启联网确认）。
- 最终 20 秒 BCW2/220 字节遥测，舍去首秒后 3802 帧，约 199.994 Hz；丢序号 0、故障样本 0、运行样本 0。
- 最终 app SHA256：16055765490a375b0c95a9732e4827adcff62e0ba362b97a61455e8ce727c7b9。
- 最终 ELF SHA256：6b23a64914f951e14a683fa47b95de30fda3ae854b4a65ae1e097253bc3ce538。

证据：acceptance.json、post_ota_telemetry.json、usb_migration.json，以及 wifi_debug/results/build_identity.json。没有做实际断电或拔 USB 验证。
