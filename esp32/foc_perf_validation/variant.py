"""Select controlled variants of the same instrumented main core."""
from pathlib import Path
import subprocess, sys
root=Path(__file__).resolve().parents[2]
assert subprocess.check_output(['git','branch','--show-current'],cwd=root,text=True).strip()=='codex/test-main-foc-perf-240-lut'
name=sys.argv[1]
assert name in ['baseline','cpu240','sqrt','wrap','sincos','lut']
base=root/'esp32/balance_car'
s=(Path(__file__).parent/'baseline_motor.cpp').read_text(encoding='utf-8')
s=s.replace('#include "sensor.h"','#include "sensor.h"\n#include "foc_math.h"')
if name!='baseline':
    s=s.replace('constexpr uint32_t CPU_MHZ = 160;','constexpr uint32_t CPU_MHZ = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;')
    s=s.replace('    constexpr uint32_t PERIOD = 8000;','    constexpr uint32_t PERIOD = 8000;\n    constexpr uint32_t CYCLES_PER_TICK = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ / 80;')
    s=s.replace('(esp_cpu_get_cycle_count() - cycles + 1U) / 2U','(esp_cpu_get_cycle_count() - cycles + CYCLES_PER_TICK - 1U) / CYCLES_PER_TICK')
if name in ['sqrt','wrap','sincos','lut']:
    s=s.replace('const float magnitude = sqrtf(vd * vd + vq * vq);\n\n            if(magnitude > limit)\n            {\n                const float scale = limit / magnitude;',
        'const float magnitude_squared = vd * vd + vq * vq;\n\n            if(magnitude_squared > limit * limit)\n            {\n                const float scale = limit / sqrtf(magnitude_squared);')
if name in ['wrap','sincos','lut']:
    s=s.replace('angle = fmodf(angle, TWO_PI);\n            return angle < 0.0f ? angle + TWO_PI : angle;', 'return foc_math::wrap(angle);')
if name in ['sincos','lut']:
    s=s.replace('const float cs = cosf(theta);\n        const float sn = sinf(theta);',
        'float cs = 0.0f;\n        float sn = 0.0f;\n        '+('sincosf(theta, &sn, &cs);' if name=='sincos' else 'foc_math::sincos(theta, sn, cs);'))
if name=='lut':
    s=s.replace('        adc::init();','        adc::init();\n        foc_math::init();')
(base/'user_lib/hw/motor.cpp').write_text(s,encoding='utf-8')
config=subprocess.check_output(['git','show','main:esp32/balance_car/sdkconfig'],cwd=root).decode()
if name!='baseline':
    for prefix in ['CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_', 'CONFIG_ESP32S3_DEFAULT_CPU_FREQ_']:
        config=config.replace(prefix+'160=y','# '+prefix+'160 is not set').replace('# '+prefix+'240 is not set',prefix+'240=y')
    config=config.replace('CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ=160','CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ=240').replace('CONFIG_ESP32S3_DEFAULT_CPU_FREQ_MHZ=160','CONFIG_ESP32S3_DEFAULT_CPU_FREQ_MHZ=240')
(base/'sdkconfig').write_text(config,encoding='utf-8')
print('Selected:',name)
