#include "test.h"
#include "hw/sensor.h"
#include "foc_current.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <inttypes.h>

namespace test { namespace {
void timing(const char*name,const foc_current::timing&t,uint32_t n){
 printf(",\"%s_us\":[%.4f,%.4f,%.4f]",name,n?double(t.sum)/n/160:0,t.minimum==UINT32_MAX?0:double(t.minimum)/160,double(t.maximum)/160);
}
void observe(void*){
 while(true){
  vTaskDelay(pdMS_TO_TICKS(1000));
  foc_current::state s{};foc_current::get_package(s);
  sensor::package p{};bool ready=sensor::get_package(p);int64_t now=esp_timer_get_time();
  printf("\nYIELD_JSON {\"time_us\":%" PRIi64 ",\"priority\":%u,\"foc\":%u,\"events\":%u,\"idle\":%u,\"invalid\":%u,\"cmp_errors\":%u,\"gaps\":%u,\"late_half\":%u,\"ready\":%d",now,unsigned(foc_current::TASK_PRIORITY),unsigned(s.count),unsigned(s.events),unsigned(s.idle),unsigned(s.adc_invalid),unsigned(s.compare_errors),unsigned(s.gaps),unsigned(s.late_half),ready);
  printf(",\"sensor_age_us\":[%" PRIi64 ",%" PRIi64 ",%" PRIi64 "]",p.left_encoder.timestamp_us?now-int64_t(p.left_encoder.timestamp_us):-1,p.right_encoder.timestamp_us?now-int64_t(p.right_encoder.timestamp_us):-1,p.imu.timestamp_us?now-int64_t(p.imu.timestamp_us):-1);
  printf(",\"samples\":[%u,%u,%u,%u,%u],\"fresh\":[%u,%u],\"reused\":[%u,%u],\"skipped\":[%u,%u],\"bus_v\":%.4f",unsigned(s.samples[0]),unsigned(s.samples[1]),unsigned(s.samples[2]),unsigned(s.samples[3]),unsigned(s.samples[4]),unsigned(s.fresh[0]),unsigned(s.fresh[1]),unsigned(s.reused[0]),unsigned(s.reused[1]),unsigned(s.skipped[0]),unsigned(s.skipped[1]),s.bus_V);
  printf(",\"event_interval_us\":[%.4f,%.4f]",s.interval_min==UINT32_MAX?0:double(s.interval_min)/160,double(s.interval_max)/160);
  timing("pair",s.acquisition,s.events);timing("phase",s.phase,s.events);timing("skew",s.skew,s.events);timing("bus",s.bus,s.samples[4]);
  timing("calc",s.calculation,s.count);timing("write",s.write,s.count);timing("work",s.total,s.count);timing("dt",s.dt,s.count);timing("age_m1",s.age[0],s.count);timing("age_m2",s.age[1],s.count);
  printf(",\"raw_mean\":[");for(unsigned i=0;i<5;++i)printf("%s%.3f",i?",":"",s.samples[i]?double(s.raw_sum[i])/s.samples[i]:0);
  printf("],\"raw_range\":[");for(unsigned i=0;i<5;++i)printf("%s[%u,%u]",i?",":"",unsigned(s.raw_min[i]),unsigned(s.raw_max[i]));
  printf("]}\n");fflush(stdout);
 }
}
void start(void*){
 gpio_set_level(GPIO_NUM_7,1);gpio_set_direction(GPIO_NUM_7,GPIO_MODE_OUTPUT);vTaskDelay(pdMS_TO_TICKS(300));
 xTaskCreatePinnedToCore(observe,"timing_observe",6144,nullptr,1,nullptr,0);
 foc_current::init();vTaskDelete(nullptr);
}
}
bool init(){return xTaskCreatePinnedToCore(start,"foc_start",4096,nullptr,1,nullptr,1)==pdPASS;}
}
