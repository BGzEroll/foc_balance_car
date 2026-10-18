from pathlib import Path

r=Path(__file__).parent
s=(r/'before.cpp').read_text(encoding='utf-8')
h=(r/'before.h').read_text(encoding='utf-8')
t=(r/'test_before.cpp').read_text(encoding='utf-8')
for old in [
 '#include "esp_freertos_hooks.h"\n',
 'volatile uint32_t idle_runs=0;\n',
 'bool idle_hook(){__atomic_add_fetch(&idle_runs,1,__ATOMIC_RELAXED);taskYIELD();return false;}\n',
 ' out.idle=__atomic_load_n(&idle_runs,__ATOMIC_RELAXED);\n',
 ' ESP_ERROR_CHECK(esp_register_freertos_idle_hook_for_cpu(idle_hook,1));\n',
]:
 assert old in s,old
 s=s.replace(old,'',1)
h=h.replace('TASK_PRIORITY=0','TASK_PRIORITY=5').replace(',idle=0;', ';')
t=t.replace('#include "esp_timer.h"','#include "esp_timer.h"\n#include "esp_task_wdt.h"\n#include "esp_heap_caps.h"\n#include "sdkconfig.h"')
t=t.replace(',\\"idle\\":%u','').replace(',unsigned(s.idle)','')
needle='  printf(",\\"sensor_age_us\\"'
idx=t.index(needle)
t=t[:idx]+'''  const esp_err_t idle0_status=esp_task_wdt_status(xTaskGetIdleTaskHandleForCore(0));
  const esp_err_t idle1_status=esp_task_wdt_status(xTaskGetIdleTaskHandleForCore(1));
  printf(",\\\"idle0_wdt_status\\\":%d,\\\"idle1_wdt_status\\\":%d,\\\"iwdt_core1\\\":%d,\\\"free_heap\\\":%u,\\\"min_free_heap\\\":%u",int(idle0_status),int(idle1_status),
#ifdef CONFIG_ESP_INT_WDT_CHECK_CPU1
         1,
#else
         0,
#endif
         unsigned(heap_caps_get_free_size(MALLOC_CAP_8BIT)),unsigned(heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT)));
'''+t[idx:]
for name,text in [('foc_current.cpp',s),('foc_current.h',h),('test.cpp',t)]:
 (r/name).write_text('\n'.join(line.rstrip() for line in text.splitlines())+'\n',encoding='utf-8',newline='\n')
(r/'sdkconfig.defaults').write_text('''# Dedicated core1 runs sensor_foc continuously; monitor control progress separately.
CONFIG_ESP_TASK_WDT_EN=y
CONFIG_ESP_TASK_WDT_INIT=y
CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0=y
# CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1 is not set
CONFIG_ESP_INT_WDT=y
CONFIG_ESP_INT_WDT_CHECK_CPU1=y
''',encoding='utf-8',newline='\n')
