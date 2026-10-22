raise SystemExit("Historical intermediate generator; do not run. Use the current checked-in test firmware and remote.py.")
"""Restore the original main core plus explicit, reviewable bench/integration fixes.

Never changes main or Git refs. Experimental scheduler rewrites are excluded.
"""
from pathlib import Path
import subprocess
root=Path(__file__).resolve().parents[2]
assert subprocess.check_output(['git','branch','--show-current'],cwd=root,text=True).strip()!='main'
assert subprocess.check_output(['git','rev-parse','main'],cwd=root,text=True).strip()=='fcaeb92cf4a849c5c7f4735986596b2ae573d460'
def original(path):return subprocess.check_output(['git','show','main:'+path],cwd=root).decode()
path='esp32/balance_car/user_lib/hw/motor.cpp';s=original(path)
def replace(a,b):
 global s
 assert s.count(a)==1,(a,s.count(a))
 s=s.replace(a,b)
replace('#include "hal/adc_ll.h"','#pragma GCC diagnostic push\n#pragma GCC diagnostic ignored "-Wmissing-field-initializers"\n#include "hal/adc_ll.h"\n#pragma GCC diagnostic pop')
replace('namespace pwm\n{','namespace pwm\n{\n    uint32_t bench_deadline_us = 0;')
replace('uint32_t fault_us = 0;','uint32_t fault_us = 0;\n        uint32_t fault_pc = 0;')
replace('void IRAM_ATTR trip(','void IRAM_ATTR __attribute__((noinline)) trip(')
replace('            __atomic_store_n(&fault_motor, index, __ATOMIC_RELAXED);','            fault_pc = reinterpret_cast<uint32_t>(__builtin_return_address(0));\n            __atomic_store_n(&fault_motor, index, __ATOMIC_RELAXED);')
replace('        out.code = get_fault_code();','        out.code = get_fault_code();\n        out.active = out.code != motor::fault_code::NONE;')
replace('        if(wanted)\n        {','        if(wanted)\n        {\n            if(bench_deadline_us && static_cast<int32_t>(now - bench_deadline_us) >= 0)\n            {\n                stop();\n                return false;\n            }')
replace('if(now - __atomic_load_n(&heartbeat_us, __ATOMIC_ACQUIRE) > 3000)','if(static_cast<int32_t>(now - __atomic_load_n(&heartbeat_us, __ATOMIC_ACQUIRE)) > 3000)')
replace('                    (same_cycle ? 32U : 0U);','                    (same_cycle ? 32U : 0U);\n#ifdef MOTOR_SAMPLE_HOOK\n        MOTOR_SAMPLE_HOOK(index, out, output_enabled);\n#endif')
replace('constexpr float VOLTAGE_LIMIT = 0.12f;','float voltage_limit = 0.12f;')
replace('constexpr float BUS_LIMIT_RATIO = 0.015f;','float bus_limit_ratio = 0.015f;\n        bool low_side_clamp = false;')
replace('fminf(VOLTAGE_LIMIT, BUS_LIMIT_RATIO * bus)','fminf(voltage_limit, bus_limit_ratio * bus)')
replace('''                const float value =
                    (0.5f + (phase[i] + common) / bus) * 4000.0f + 0.5f;''','''                // Bench wide mode: equal common-mode shift preserves line
                // voltages; ratio <= .24 keeps compares <= 1983 (< neutral).
                const float value = low_side_clamp
                    ? 320.0f + (phase[i] - valley) / bus * 4000.0f + 0.5f
                    : (0.5f + (phase[i] + common) / bus) * 4000.0f + 0.5f;''')
replace('        output = modulation(vd, vq, theta, bus);','''        output = modulation(vd, vq, theta, bus);
        info.vd_V = vd;
        info.vq_V = vq;
        info.voltage_limit_V = fminf(voltage_limit, bus_limit_ratio * bus);
#ifdef MOTOR_CONTROL_HOOK
        if(!aligning) { MOTOR_CONTROL_HOOK(index, info, target); }
#endif''')
replace('            RUN\n','            RUN,\n            IDLE\n')
replace('            stage state = stage::WAITING;','''#ifdef MOTOR_BENCH_MANUAL_ARM
            stage state = stage::IDLE;
#else
            stage state = stage::WAITING;
#endif''')
replace('''        void calibrate(const sensor::package &sensors, uint32_t now, uint64_t now64)
        {''','''        void calibrate(const sensor::package &sensors, uint32_t now, uint64_t now64)
        {
            if(calibration.state == stage::IDLE) { return; }''')
replace('''                const bool bus_valid = bus_us != 0 &&
                    now - bus_us <= pwm::BUS_TIMEOUT_US &&''','''                const bool bus_valid = bus_us != 0 &&
                    now32() - bus_us <= pwm::BUS_TIMEOUT_US &&''')
replace('''                    if(!fresh(encoders[i], now64))''','''                    if(pwm::is_requested() && (pwm::mask & (1U << i)) &&
                       !fresh(encoders[i], now64))''')
replace('''                    if(!pwm::start(3U))
                    {
                        pwm::trip(fault_code::ALIGNMENT);
                    }''','''#ifndef MOTOR_BENCH_MANUAL_ARM
                    if(!pwm::start(3U))
                    {
                        pwm::trip(fault_code::ALIGNMENT);
                    }
#endif''')
replace('if(now - sample.timestamp_us > pwm::SAMPLE_TIMEOUT_US)','if(now32() - sample.timestamp_us > pwm::SAMPLE_TIMEOUT_US)')
replace('''            pwm::get_bus(bus_raw, bus_stamp);

            if(bus_stamp''','''            pwm::get_bus(bus_raw, bus_stamp);
            now = now32();

            if(bus_stamp''')
replace('''                const uint32_t now = now32();
                const uint64_t now64 = static_cast<uint64_t>(esp_timer_get_time());
                pwm::heartbeat(now);

                sensor::package sensors{};
                sensor::get_package(sensors);''','''                sensor::package sensors{};
                sensor::get_package(sensors);
                const uint64_t now64 = static_cast<uint64_t>(esp_timer_get_time());
                const uint32_t now = static_cast<uint32_t>(now64);
                pwm::heartbeat(now);''')
replace('''                    calibration.seen_sequence[index] = sample.sequence;
''','''                    calibration.seen_sequence[index] = sample.sequence;
                    if(!(sample.flags & 1U) || !pwm::is_running()) { continue; }
''')
replace('        constexpr float MAX_TARGET_A = 2.0f;','        constexpr float MAX_TARGET_A = 2.0f;\n        float speed_trip_rad_s = 40.0f;')
replace('fabsf(encoder.speed_rad_s) > 40.0f','fabsf(encoder.speed_rad_s) > speed_trip_rad_s')
(root/path).write_text(s,encoding='utf-8')
hpath='esp32/balance_car/user_lib/hw/motor.h';h=original(hpath)
h=h.replace('        bool valid = false;','        bool valid = false;\n        float vd_V = 0.0f;\n        float vq_V = 0.0f;\n        float voltage_limit_V = 0.0f;')
(root/hpath).write_text(h,encoding='utf-8')
cfg=original('esp32/balance_car/sdkconfig').replace('# CONFIG_MCPWM_CTRL_FUNC_IN_IRAM is not set','CONFIG_MCPWM_CTRL_FUNC_IN_IRAM=y')
for key in ['CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1','CONFIG_TASK_WDT_CHECK_IDLE_TASK_CPU1']:
 cfg=cfg.replace(key+'=y','# '+key+' is not set')
(root/'esp32/balance_car/sdkconfig').write_text(cfg,encoding='utf-8')
cmake=root/'esp32/balance_car/main/CMakeLists.txt'
c=cmake.read_text().split('# Keep the extracted real-time')[0].rstrip()+'\n';cmake.write_text(c)
bench=root/'esp32/balance_car/user_lib/foc_bench.cpp';b=bench.read_text()
if '    if(s.fault.active){printf("PROFILE")' in b:
 begin=b.index('    if(s.fault.active){printf("PROFILE")');end=b.index('    printf("STATUS ',begin)
 b=b[:begin]+b[end:]
b=b.replace(',unsigned(eTaskGetState(pwm::control_task))',',unsigned(0)')
b=b.replace(';pwm::inspected[i]=pwm::processed[i]=pwm::rejected[i]=0','')
b=b.replace('    pwm::notify_sent=pwm::notify_wake=pwm::notify_received=pwm::notify_bits=0;\n','')
b=b.replace('motor::calibration.state==motor::stage::RUN&&pwm::is_running()',
            'motor::calibration.state==motor::stage::RUN&&foc::motors[0].aligned&&foc::motors[1].aligned')
if 'motor::speed_trip_rad_s=' not in b:
 b=b.replace('    foc::reset();foc::voltage_limit=cap;foc::bus_limit_ratio=ratio;',
             '    foc::reset();foc::voltage_limit=cap;foc::bus_limit_ratio=ratio;\n    motor::speed_trip_rad_s=std::max(40.0f,std::min(120.0f,fabsf(x)*1.2f+10.0f));')
bench.write_text(b,encoding='utf-8')
print('Restored original main FOC path; v18 polling/priority/core/IRAM/Idle1 settings; bench stopped by default.')
