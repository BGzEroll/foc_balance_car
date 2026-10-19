#include "test.h"
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
