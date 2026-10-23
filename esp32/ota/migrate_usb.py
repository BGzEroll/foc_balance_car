"""One-time, stopped USB migration to 8 MB factory + two OTA slots. Keeps NVS."""
import argparse
import contextlib
from datetime import datetime, timezone
import hashlib
import io
import json
from pathlib import Path
import struct
import subprocess
import sys

from client import ROOT, HERE, image_info, stopped
sys.path.insert(0, str(ROOT / 'esp32/wifi_debug/.tools'))
import esptool

BASE = '9faaf7f22f017444cb60e80a31227c53518281e4'
BRANCH = 'codex/test-upper-controller'
FW = ROOT / 'esp32/wifi_debug/firmware'
BACKUP = FW / 'pre_ota_8MB.bin'


def guard():
    assert subprocess.check_output(['git', 'branch', '--show-current'], cwd=ROOT, text=True).strip() == BRANCH
    assert subprocess.check_output(['git', 'rev-parse', 'main'], cwd=ROOT, text=True).strip() == BASE


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def identity():
    names = ['CMakeLists.txt', 'main/CMakeLists.txt', 'sdkconfig', 'user_lib/app.cpp',
             'user_lib/balance.cpp', 'user_lib/balance.h', 'user_lib/test.cpp', 'user_lib/test.h',
             'user_lib/wifi_debug.cpp', 'user_lib/wifi_debug.h', 'user_lib/ota.cpp', 'user_lib/ota.h',
             'user_lib/hw/motor.cpp', 'user_lib/hw/motor.h', '../tuning_preparation/ota_partitions_8mb.csv']
    return dict(branch=BRANCH, main_before=BASE, main_after=BASE,
                **{name: digest(FW/name) for name in ['balance_car.bin', 'bootloader.bin', 'partition-table.bin']},
                source_sha256={name: digest(ROOT/'esp32/balance_car'/name) for name in names},
                application=image_info((FW/'balance_car.bin').read_bytes()))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', default='COM10')
    p.add_argument('--host', default='192.168.114.243')
    p.add_argument('--baud', default='921600')
    p.add_argument('--backup-only', action='store_true')
    args = p.parse_args()
    guard()
    print(stopped(args.host), flush=True)
    config = (ROOT/'esp32/balance_car/sdkconfig').read_text()
    for flag in ['CONFIG_ESPTOOLPY_FLASHSIZE_8MB=y', 'CONFIG_PARTITION_TABLE_CUSTOM=y',
                 'CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y']:
        assert flag in config
    table = (FW/'partition-table.bin').read_bytes()
    actual = {}
    for i in range(0, len(table), 32):
        magic, kind, subtype, offset, size, label, flags = struct.unpack('<HBBII16sI', table[i:i+32])
        if magic != 0x50AA:
            break
        actual[label.split(b'\0')[0].decode()] = (offset, size)
    expected = {'nvs': (0x9000,0x6000), 'phy_init': (0xf000,0x1000), 'factory': (0x10000,0x180000),
                'otadata': (0x190000,0x2000), 'ota_0': (0x1a0000,0x200000), 'ota_1': (0x3a0000,0x200000)}
    assert actual == expected, actual
    image_info((FW/'balance_car.bin').read_bytes())
    command = ['--chip','esp32s3','--port',args.port,'--baud',args.baud]
    # Verify physical identity/capacity before any writes, even when a backup already exists.
    captured = io.StringIO()
    with contextlib.redirect_stdout(captured):
        esptool.main(command + ['flash-id'])
    evidence = captured.getvalue()
    assert '34:cd:b0:38:ac:90' in evidence.lower(), 'Unexpected board MAC'
    assert '8MB' in evidence.replace(' ', ''), 'Physical flash is not 8 MB'
    print('CHIP_OK ESP32-S3 MAC 34:CD:B0:38:AC:90 flash=8MB', flush=True)
    backup_manifest = FW/'pre_ota_8MB.json'
    if not BACKUP.exists():
        partial = FW/'pre_ota_8MB.partial'
        esptool.main(command + ['read-flash','0','0x800000',str(partial)])
        assert partial.stat().st_size == 0x800000
        partial.rename(BACKUP)
        (FW/'pre_ota_nvs.bin').write_bytes(BACKUP.read_bytes()[0x9000:0xf000])
        backup_manifest.write_text(json.dumps(dict(mac='34:CD:B0:38:AC:90', bytes=0x800000,
            sha256=digest(BACKUP), nvs_sha256=digest(FW/'pre_ota_nvs.bin'),
            utc=datetime.now(timezone.utc).isoformat()),indent=2))
    manifest=json.loads(backup_manifest.read_text())
    assert BACKUP.stat().st_size == 0x800000 and digest(BACKUP) == manifest['sha256']
    assert manifest['mac'] == '34:CD:B0:38:AC:90'
    print('BACKUP_OK full 8MB and NVS retained locally', flush=True)
    if args.backup_only:
        return
    result = identity()
    # Only otadata is erased. Preserve NVS/PHY and leave factory as the rescue application.
    esptool.main(command + ['erase-region','0x190000','0x2000'])
    esptool.main(command + ['write-flash','--flash-mode','dio','--flash-size','8MB','--flash-freq','80m',
        '0x0',str(FW/'bootloader.bin'),'0x8000',str(FW/'partition-table.bin'),'0x10000',str(FW/'balance_car.bin')])
    guard()
    result.update(migration_utc=datetime.now(timezone.utc).isoformat(), backup=manifest, partitions=actual)
    out = HERE/'results';out.mkdir(exist_ok=True)
    (out/'usb_migration.json').write_text(json.dumps(result,indent=2))
    (ROOT/'esp32/wifi_debug/results/build_identity.json').write_text(json.dumps(result,indent=2))
    print('MIGRATION_OK; main unchanged; booting stopped factory OTA service', flush=True)


if __name__ == '__main__':
    main()
