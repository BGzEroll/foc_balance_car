#include "test.h"
#include "hw/sensor.h"
#include "foc_current.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "esp_heap_caps.h"
#include "sdkconfig.h"
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
  printf("\nYIELD_JSON {\"time_us\":%" PRIi64 ",\"priority\":%u,\"foc\":%u,\"events\":%u,\"invalid\":%u,\"cmp_errors\":%u,\"gaps\":%u,\"late_half\":%u,\"ready\":%d",now,unsigned(foc_current::TASK_PRIORITY),unsigned(s.count),unsigned(s.events),unsigned(s.adc_invalid),unsigned(s.compare_errors),unsigned(s.gaps),unsigned(s.late_half),ready);
  const esp_err_t idle0_status=esp_task_wdt_status(xTaskGetIdleTaskHandleForCore(0));
  const esp_err_t idle1_status=esp_task_wdt_status(xTaskGetIdleTaskHandleForCore(1));
  printf(",\"idle0_wdt_status\":%d,\"idle1_wdt_status\":%d,\"iwdt_core1\":%d,\"free_heap\":%u,\"min_free_heap\":%u",int(idle0_status),int(idle1_status),
#ifdef CONFIG_ESP_INT_WDT_CHECK_CPU1
         1,
#else
         0,
#endif
         unsigned(heap_caps_get_free_size(MALLOC_CAP_8BIT)),unsigned(heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT)));
  printf(",\"sensor_age_us\":[%" PRIi64 ",%" PRIi64 ",%" PRIi64 "]",p.left_encoder.timestamp_us?now-int64_t(p.left_encoder.timestamp_us):-1,p.right_encoder.timestamp_us?now-int64_t(p.right_encoder.timestamp_us):-1,p.imu.timestamp_us?now-int64_t(p.imu.timestamp_us):-1);
  printf(",\"samples\":[%u,%u,%u,%u,%u],\"fresh\":[%u,%u],\"reused\":[%u,%u],\"skipped\":[%u,%u],\"bus_v\":%.4f",unsigned(s.samples[0]),unsigned(s.samples[1]),unsigned(s.samples[2]),unsigned(s.samples[3]),unsigned(s.samples[4]),unsigned(s.fresh[0]),unsigned(s.fresh[1]),unsigned(s.reused[0]),unsigned(s.reused[1]),unsigned(s.skipped[0]),unsigned(s.skipped[1]),s.bus_V);
  printf(",\"event_interval_us\":[%.4f,%.4f]",s.interval_min==UINT32_MAX?0:double(s.interval_min)/160,double(s.interval_max)/160);
  printf(",\"rejected\":[%u,%u]",unsigned(s.rejected[0]),unsigned(s.rejected[1]));
  printf(",\"late_atomic\":%u,\"phase_disagree\":%u",unsigned(s.late_atomic),unsigned(s.phase_disagree));
  timing("atomic_phase",s.atomic_phase,s.events);timing("completion",s.completion,s.events);
  timing("pair",s.acquisition,s.events);timing("phase",s.phase,s.events);timing("skew",s.skew,s.events);timing("bus",s.bus,s.samples[4]);
  timing("calc",s.calculation,s.updates[0]+s.updates[1]);timing("write",s.write,s.updates[0]+s.updates[1]);
  timing("work",s.total,s.loop_count);timing("dt",s.dt,s.loop_count);
  timing("age_m1",s.age[0],s.updates[0]);timing("age_m2",s.age[1],s.updates[1]);
  timing("sample_dt_m1",s.sample_dt[0],s.updates[0]);timing("sample_dt_m2",s.sample_dt[1],s.updates[1]);
  printf(",\"loops\":%u,\"no_update_loops\":%u,\"updates\":[%u,%u],\"pi_calls\":[%u,%u],\"prime_count\":[%u,%u],\"bad_sample_dt\":[%u,%u]",unsigned(s.loop_count),unsigned(s.no_update_loops),unsigned(s.updates[0]),unsigned(s.updates[1]),unsigned(s.pi_calls[0]),unsigned(s.pi_calls[1]),unsigned(s.prime_count[0]),unsigned(s.prime_count[1]),unsigned(s.bad_sample_dt[0]),unsigned(s.bad_sample_dt[1]));
  printf(",\"integral_d\":[%.6f,%.6f],\"integral_q\":[%.6f,%.6f]",s.integral_d[0],s.integral_d[1],s.integral_q[0],s.integral_q[1]);
  printf(",\"raw_mean\":[");for(unsigned i=0;i<5;++i)printf("%s%.3f",i?",":"",s.samples[i]?double(s.raw_sum[i])/s.samples[i]:0);
  printf("],\"raw_range\":[");for(unsigned i=0;i<5;++i)printf("%s[%u,%u]",i?",":"",unsigned(s.raw_min[i]),unsigned(s.raw_max[i]));
  printf("],\"phase_hist\":[");for(unsigned i=0;i<50;++i)printf("%s%u",i?",":"",unsigned(s.phase_hist[i]));printf("]}\n");fflush(stdout);
 }
}
void start(void*){
 gpio_set_level(GPIO_NUM_7,1);gpio_set_direction(GPIO_NUM_7,GPIO_MODE_OUTPUT);vTaskDelay(pdMS_TO_TICKS(300));
 xTaskCreatePinnedToCore(observe,"timing_observe",6144,nullptr,1,nullptr,0);
 foc_current::init();vTaskDelete(nullptr);
}
}
bool init(){return xTaskCreatePinnedToCore(start,"foc_start",4096,nullptr,1,nullptr,0)==pdPASS;}
}
