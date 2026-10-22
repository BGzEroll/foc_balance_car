"""Export only the requested production changes, without bench instrumentation."""
from pathlib import Path
import subprocess,difflib,json,hashlib
root=Path(__file__).resolve().parents[2]
out=Path(__file__).resolve().parent
def original(path):return subprocess.check_output(['git','show','main:'+path],cwd=root).decode().replace('\r\n','\n')
motor_path='esp32/balance_car/user_lib/hw/motor.cpp'
s=original(motor_path)
s=s.replace('#include "sensor.h"','#include "sensor.h"\n#include "foc_math.h"')
s=s.replace('constexpr uint32_t CPU_MHZ = 160;','constexpr uint32_t CPU_MHZ = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;')
s=s.replace('    constexpr uint32_t PERIOD = 8000;','    constexpr uint32_t PERIOD = 8000;\n    constexpr uint32_t CYCLES_PER_TICK = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ / 80;')
s=s.replace('(esp_cpu_get_cycle_count() - cycles + 1U) / 2U','(esp_cpu_get_cycle_count() - cycles + CYCLES_PER_TICK - 1U) / CYCLES_PER_TICK')
s=s.replace('angle = fmodf(angle, TWO_PI);\n            return angle < 0.0f ? angle + TWO_PI : angle;','return foc_math::wrap(angle);')
s=s.replace('const float cs = cosf(theta);\n        const float sn = sinf(theta);','float cs = 0.0f;\n        float sn = 0.0f;\n        foc_math::sincos(theta, sn, cs);')
s=s.replace('const float magnitude = sqrtf(vd * vd + vq * vq);\n\n            if(magnitude > limit)\n            {\n                const float scale = limit / magnitude;','const float magnitude_squared = vd * vd + vq * vq;\n\n            if(magnitude_squared > limit * limit)\n            {\n                const float scale = limit / sqrtf(magnitude_squared);')
s=s.replace('        adc::init();','        adc::init();\n        foc_math::init();')
config_path='esp32/balance_car/sdkconfig'
c=original(config_path)
for prefix in ['CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_','CONFIG_ESP32S3_DEFAULT_CPU_FREQ_']:
    c=c.replace(prefix+'160=y','# '+prefix+'160 is not set').replace('# '+prefix+'240 is not set',prefix+'240=y')
c=c.replace('CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ=160','CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ=240').replace('CONFIG_ESP32S3_DEFAULT_CPU_FREQ_MHZ=160','CONFIG_ESP32S3_DEFAULT_CPU_FREQ_MHZ=240')
helper_path='esp32/balance_car/user_lib/hw/foc_math.h'
contents={motor_path:s,config_path:c,helper_path:(root/helper_path).read_text(encoding='utf-8').replace('\r\n','\n')}
patch=''
for path,text in contents.items():
    old='' if path==helper_path else original(path)
    patch+=''.join(difflib.unified_diff(old.splitlines(True),text.splitlines(True),fromfile='/dev/null' if path==helper_path else 'a/'+path,tofile='b/'+path))
(out/'core_changes.patch').write_text(patch,encoding='utf-8',newline='\n')
manifest={'base_main':subprocess.check_output(['git','rev-parse','main'],cwd=root,text=True).strip(),'production_source_sha256_lf':{p:hashlib.sha256(t.encode()).hexdigest() for p,t in contents.items()},'note':'Production-only patch. Hardware bench wraps this same math and timing code; no main mutation.'}
(out/'production_manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
print('Exported core_changes.patch: motor.cpp, sdkconfig, foc_math.h only.')
