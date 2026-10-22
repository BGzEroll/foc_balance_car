"""Prepare a finite bench around latest-main motor.cpp; main ref is never changed."""
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
BRANCH = 'codex/test-main-foc-perf-240-lut'
assert subprocess.check_output(['git','branch','--show-current'],cwd=ROOT,text=True).strip()==BRANCH
def source(ref, path):
    return subprocess.check_output(['git','show',ref+':'+path],cwd=ROOT).decode().replace('\r\n','\n')
base=source('main','esp32/balance_car/user_lib/hw/motor.cpp')
for name in ['run_new.py','run_reference.py','capture.py','analyze.py','gate.py']:
    (OUT/name).write_text(source('codex/test-main-foc-validation','esp32/main_foc_validation/'+name),encoding='utf-8')
old='codex/test-main-foc-validation'
bench=source(old,'esp32/balance_car/user_lib/foc_bench.cpp')
bench=bench.replace('ratio>(foc::low_side_clamp?.24f:.04f)','ratio>.08f')
bench=bench.replace('    if(!strcmp(line,"WIDE")) {','    if(!strcmp(line,"WIDE")) {')
bench=bench.replace('        foc::low_side_clamp=true;puts("WIDE_OK");return;','        puts("ERROR unsupported");return;')
bench=bench.replace('void foc_bench_init() {','void foc_bench_init() {\n    extern void math_benchmark(); math_benchmark();')
bench=bench.replace('    puts("END");','    for(unsigned m=0;m<2;++m) printf("TIMING {\\"motor\\":%u,\\"compute_max_us\\":%u,\\"command_age_max_us\\":%u}\\n",m+1,foc::motors[m].max_compute_us,foc::motors[m].max_command_age_us);\n    puts("END");')
bench += '\n#include "math_benchmark.inc"\n'
bench=bench.replace('foc::motors[m].max_compute_us,foc::motors[m].max_command_age_us','unsigned(foc::motors[m].max_compute_us),unsigned(foc::motors[m].max_command_age_us)')
(ROOT/'esp32/balance_car/user_lib/perf_bench.cpp').write_text(bench,encoding='utf-8')
def replace(a,b):
    global base
    assert base.count(a)==1, (a,base.count(a))
    base=base.replace(a,b)
replace('namespace pwm\n{','namespace pwm\n{\n    uint32_t bench_deadline_us = 0;')
replace('        uint32_t fault_us = 0;','        uint32_t fault_us = 0;\n        uint32_t fault_pc = 0;')
replace('void IRAM_ATTR trip(','void IRAM_ATTR __attribute__((noinline)) trip(')
replace('            __atomic_store_n(&fault_motor, index, __ATOMIC_RELAXED);','            fault_pc = reinterpret_cast<uint32_t>(__builtin_return_address(0));\n            __atomic_store_n(&fault_motor, index, __ATOMIC_RELAXED);')
replace('        if(wanted)\n        {','        if(wanted)\n        {\n            if(bench_deadline_us && static_cast<int32_t>(now - bench_deadline_us) >= 0)\n            {\n                stop();\n                return false;\n            }')
replace('        const bool over = adc::overcurrent(index, a, b);','#ifdef MOTOR_SAMPLE_HOOK\n        MOTOR_SAMPLE_HOOK(index, out, output_enabled);\n#endif\n\n        const bool over = adc::overcurrent(index, a, b);')
replace('        constexpr float BUS_LIMIT_RATIO = 0.08f;','        float voltage_limit = 0.652f;\n        float bus_limit_ratio = 0.08f;')
replace('            uint32_t last_sample_us = 0;','            uint32_t last_sample_us = 0;\n            uint32_t updates = 0, max_dt_us = 0;\n            uint32_t max_compute_us = 0, max_command_age_us = 0;')
replace('            motors[i].last_sample_us = 0;','            motors[i].last_sample_us = 0;\n            motors[i].updates = motors[i].max_dt_us = 0;\n            motors[i].max_compute_us = motors[i].max_command_age_us = 0;')
replace('            const float dt = dt_us * 1.0e-6f;','            ++motors[index].updates;\n            if(dt_us > motors[index].max_dt_us) { motors[index].max_dt_us = dt_us; }\n            const float dt = dt_us * 1.0e-6f;')
replace('const float limit = BUS_LIMIT_RATIO * bus;','const float limit = fminf(voltage_limit, bus_limit_ratio * bus);')
replace('        output = modulation(vd, vq, cs, sn, bus);','        info.vd_V = vd;\n        info.vq_V = vq;\n        info.voltage_limit_V = fminf(voltage_limit, bus_limit_ratio * bus);\n        output = modulation(vd, vq, cs, sn, bus);')
replace('        constexpr float MAX_TARGET_A = 2.0f;','        constexpr float MAX_TARGET_A = 2.0f;\n        float speed_trip_rad_s = 40.0f;')
replace('            RUN\n','            RUN,\n            IDLE\n')
replace('            stage state = stage::WAITING;','#ifdef MOTOR_BENCH_MANUAL_ARM\n            stage state = stage::IDLE;\n#else\n            stage state = stage::WAITING;\n#endif')
replace('            const sensor::encoder_data encoders[2] =','            if(calibration.state == stage::IDLE) { return; }\n            const sensor::encoder_data encoders[2] =')
replace('                    if(!pwm::start(3U))\n                    {\n                        pwm::trip(fault_code::ALIGNMENT);\n                    }','#ifndef MOTOR_BENCH_MANUAL_ARM\n                    if(!pwm::start(3U))\n                    {\n                        pwm::trip(fault_code::ALIGNMENT);\n                    }\n#endif')
replace('            if(!foc::update(','            const uint32_t compute_began_us = now32();\n            if(!foc::update(')
replace('            pwm::set_command(index, output);','            pwm::set_command(index, output);\n            if(!aligning)\n            {\n                const uint32_t ended_us = now32();\n                auto &state = foc::motors[index];\n                state.max_compute_us = fmaxf(state.max_compute_us, ended_us - compute_began_us);\n                state.max_command_age_us = fmaxf(state.max_command_age_us, ended_us - sample.timestamp_us);\n            }')
replace('fabsf(encoder.speed_rad_s) > 40.0f','fabsf(encoder.speed_rad_s) > speed_trip_rad_s')
(OUT/'baseline_motor.cpp').write_text(base,encoding='utf-8')
(ROOT/'esp32/balance_car/user_lib/hw/motor.cpp').write_text(base,encoding='utf-8')
header=source('main','esp32/balance_car/user_lib/hw/motor.h')
header=header.replace('        bool valid = false;','        bool valid = false;\n        float vd_V = 0.0f, vq_V = 0.0f, voltage_limit_V = 0.0f;')
(ROOT/'esp32/balance_car/user_lib/hw/motor.h').write_text(header,encoding='utf-8')
cmake=source('main','esp32/balance_car/main/CMakeLists.txt').replace('"../user_lib/test.cpp"','"../user_lib/perf_bench.cpp"').replace(' "../user_lib/hw/motor.cpp"','')
(ROOT/'esp32/balance_car/main/CMakeLists.txt').write_text(cmake,encoding='utf-8')
lf=source('main','esp32/balance_car/user_lib/hw/motor.lf').replace('    motor (noflash)','    motor (noflash)\n    perf_bench (noflash)')
(ROOT/'esp32/balance_car/user_lib/hw/motor.lf').write_text(lf,encoding='utf-8')
(ROOT/'esp32/balance_car/user_lib/app.cpp').write_text('extern void foc_bench_init();\n\nextern "C" void app_init(void)\n{\n    foc_bench_init();\n}\n',encoding='utf-8')
(OUT/'reference_v18.bin').write_bytes(subprocess.check_output(['git','show',old+':esp32/main_foc_validation/reference_v18.bin'],cwd=ROOT))
(OUT/'run_new.py').write_text((OUT/'run_new.py').read_text().replace("['STATUS','METRIC','META']","['STATUS','METRIC','META','TIMING','MATH']"),encoding='utf-8')
print('Prepared latest-main instrumented baseline; no math or frequency changes yet.')
