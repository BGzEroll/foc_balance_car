"""Generate an isolated v24 recorder from fixed v18. Baseline files are not edited here."""
from pathlib import Path
r=Path(__file__).resolve().parent;p=r.parent.parent/'balance_car'
h=(p/'user_lib/hw/sensor.h').read_text(encoding='utf-8')
h=h.replace('float speed_rad_s = 0.0f;', 'float speed_rad_s = 0.0f;\n        uint16_t raw_counts = 0;')
h=h.replace('float temperature = 0.0f;', 'float temperature = 0.0f;\n        int16_t raw_acc[3] = {}, raw_gyro[3] = {}, raw_temperature = 0;\n        float gyro_bias[3] = {};')
s=(p/'user_lib/hw/sensor.cpp').read_text(encoding='utf-8')
s=s.replace('data.timestamp_us = now_us;\n            data.angle_rad', 'data.raw_counts = raw;\n            data.timestamp_us = now_us;\n            data.angle_rad')
needle='''        data.timestamp_us = now_us;

        data.temperature ='''
assert needle in s
s=s.replace(needle,'''        data.timestamp_us = now_us;
        data.raw_temperature = raw_temperature;
        for(unsigned k=0;k<3;++k){data.raw_acc[k]=raw_acc[k];data.raw_gyro[k]=raw_gyro[k];data.gyro_bias[k]=gyro_offset[k];}

        data.temperature =''')
f=(p/'user_lib/foc_current.cpp').read_text(encoding='utf-8')
f=f.replace('namespace foc_current { namespace {','#include "sensor_record.inc"\nnamespace foc_current { namespace {',1)
f=f.replace('void command(char*line){','void command(char*line){\n if(sensor_record::command(line,!running&&!requested))return;',1)
f=f.replace('bool init(){','bool init(){',1)
f=f.replace('ESP_ERROR_CHECK(xTaskCreatePinnedToCore(console,','sensor_record::init(rows,sizeof(Row)*N);\n ESP_ERROR_CHECK(xTaskCreatePinnedToCore(console,',1)
f=f.replace('if(logging&&s.seq%decimate', 'if(logging&&!sensor_record::owns_buffer()&&s.seq%decimate')
f=f.replace(r'version\":18',r'version\":24').replace('DUAL_CURRENT_V18_INTERLEAVED','DUAL_CURRENT_V24_SENSOR_RECORDER')
assert r'version\":24' in f
for name,text in [('sensor.h',h),('sensor.cpp',s),('foc_sensor_recorder.cpp',f)]: (r/name).write_text(text,encoding='utf-8')
