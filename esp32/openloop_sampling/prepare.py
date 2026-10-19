"""Build the isolated MA open-loop experiment from the verified ADC baseline."""
from pathlib import Path
r=Path(__file__).resolve().parent
base=(r.parent/'no_idle_hook/foc_current.cpp').read_text(encoding='utf-8')
read=base[base.index('        bool IRAM_ATTR read_adc'):base.index('        void calculate')]
cal=base[base.index('        void init_adc()'):base.index('void IRAM_ATTR acquire')]
body=(r/'experiment.inc').read_text(encoding='utf-8')
body=body.replace('/* ADC_READ */',read).replace('/* ADC_INIT */',cal)
# ESP32's uint32_t is unsigned long. Keep 32-bit atomics and %u diagnostics
# consistently typed as unsigned int (also 32 bit on this target).
body=body.replace('uint32_t','unsigned')
(r/'foc_current.cpp').write_text(body,encoding='utf-8',newline='\n')
(r/'foc_current.h').write_text('#pragma once\nnamespace foc_current { bool init(); }\n',encoding='utf-8',newline='\n')
(r/'test.cpp').write_text('''#include "test.h"
#include "foc_current.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
namespace test {
void start(void*) {
 gpio_set_level(GPIO_NUM_7,1); gpio_set_direction(GPIO_NUM_7,GPIO_MODE_OUTPUT);
 vTaskDelay(pdMS_TO_TICKS(300)); foc_current::init(); vTaskDelete(nullptr);
}
bool init(){return xTaskCreatePinnedToCore(start,"openloop_start",4096,nullptr,1,nullptr,0)==pdPASS;}
}
''',encoding='utf-8',newline='\n')
