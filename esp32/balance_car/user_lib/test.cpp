#include "test.h"
#include "hw/motor.h"
#include "hw/sensor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_system.h"
#include <math.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

namespace test {
namespace {
constexpr float PI2 = 6.28318530718f;
constexpr unsigned N = 360;
struct Record { float t, angle, speed, iq, target, id; };
Record records[4000];
unsigned count = 0;
enum Mode { IDLE, SCAN, SPEED };
Mode mode = IDLE;
unsigned selected = 0, step = 0, ticks = 0, samples = 0;
float integral = 0, origin = 0, reference = 0, trajectory = 0, cap = 6.0f;
float sum_angle = 0, sum_speed = 0, sum_iq = 0, sum_target = 0, sum_id = 0;
float scan_seconds = 0.16f, duration = 0, speed_ref = 0;
int64_t began = 0, segment = 0, previous = 0;
uint64_t last_encoder = 0;
bool ff = false;
float position_kp = 20.0f, position_ki = 10.0f, position_kd = 0.04f;
float servo_speed = 0, servo_angle = 0;
float clamp(float x, float limit) { return fmaxf(-limit, fminf(limit, x)); }
void idle() {
    mode = IDLE; motor::enable_cogging(false); ff = false; integral = 0;
    for(unsigned i = 0; i < 2; ++i)
        motor::set_target_current(static_cast<motor::motor_id>(i), {});
}
void clear_sums() {
    samples = 0; sum_angle = sum_speed = sum_iq = sum_target = sum_id = 0;
}
void command(const char *line, const motor::foc_info &foc, const sensor::package &sensors) {
    unsigned m = 0, index = 0, enable = 0;
    float value = 0, seconds = 0, current = 0;
    if(!strcmp(line, "STOP")) { idle(); motor::stop(); puts("STOPPED"); }
    else if(!strcmp(line, "RESET")) { motor::stop(); esp_restart(); }
    else if(!strcmp(line, "IDLE")) { idle(); puts("IDLE"); }
    else if(sscanf(line, "GAINS %f %f %f", &value, &seconds, &current) == 3) {
        position_kp=value; position_ki=seconds; position_kd=current; puts("GAINS_OK");
    }
    else if(!strcmp(line, "STATUS"))
        printf("STATUS ready=%u fault=%lu bus=%.3f dir=%.0f,%.0f mode=%u ff=%u\n",
            foc.ready, static_cast<unsigned long>(foc.fault.code), foc.bus_voltage_V,
            foc.current[0].encoder_direction, foc.current[1].encoder_direction, mode, ff);
    else if(sscanf(line, "MAP %u %u %f", &m, &index, &value) == 3)
        printf("MAP %u\n", motor::set_cogging_point(m, index, value));
    else if(sscanf(line, "SCAN %u %f %f", &m, &seconds, &current) == 3 &&
            m < 2 && seconds >= 0.08f && seconds <= 0.5f && current > 0 && current <= 6.0f && foc.ready) {
        idle(); selected = m; scan_seconds = seconds; cap = current;
        const auto &enc = m == 0 ? sensors.left_encoder : sensors.right_encoder;
        origin = enc.angle_rad - enc.single_turn_rad;
        if(enc.single_turn_rad > 0.05f) origin += PI2;
        reference = origin; trajectory = enc.angle_rad; servo_angle = enc.angle_rad; servo_speed = 0; step = ticks = 0; clear_sums();
        began = segment = esp_timer_get_time(); previous = 0; mode = SCAN;
        puts("SCAN_BEGIN motor,step,direction,angle,speed,iq,target,id,samples,error");
    }
    else if(sscanf(line, "RUN %u %f %f %f %u", &m, &value, &seconds, &current, &enable) == 5 &&
            m < 2 && fabsf(value) <= 3 && seconds > 0 && seconds <= 19 &&
            current > 0 && current <= 6.0f && enable < 2 && foc.ready) {
        idle(); selected = m; speed_ref = value; duration = seconds; cap = current;
        ff = enable; motor::enable_cogging(ff); count = ticks = 0;
        began = esp_timer_get_time(); previous = 0; mode = SPEED; puts("RUN_BEGIN");
    }
    else if(!strcmp(line, "DUMP") && mode == IDLE) {
        puts("DATA_BEGIN t,angle,speed,iq,target,id");
        for(unsigned i = 0; i < count; ++i) {
            const auto &r = records[i];
            printf("D,%.4f,%.6f,%.5f,%.5f,%.5f,%.5f\n", r.t,r.angle,r.speed,r.iq,r.target,r.id);
            if(i % 8 == 0) vTaskDelay(1);
            for(unsigned m = 0; m < 2; ++m)
                motor::set_target_current(static_cast<motor::motor_id>(m), {});
        }
        puts("DATA_END");
    }
    else puts("ERROR_COMMAND");
}
void task(void *) {
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    setvbuf(stdin, nullptr, _IONBF, 0);
    char line[96]; unsigned length = 0;
    TickType_t wake = xTaskGetTickCount(); puts("ANTICOGGING_READY");
    while(true) {
        vTaskDelayUntil(&wake, 1);
        motor::foc_info foc{}; motor::get_info(foc);
        sensor::package sensors{}; sensor::get_package(sensors);
        char ch;
        while(read(STDIN_FILENO, &ch, 1) == 1) {
            if(ch == '\n' || ch == '\r') {
                if(length) { line[length] = 0; command(line, foc, sensors); wake = xTaskGetTickCount(); length = 0; }
            }
            else if(length < sizeof(line)-1) line[length++] = ch;
        }
        if(!foc.ready || foc.fault.active) {
            if(mode != IDLE) { idle(); printf("ABORT fault=%lu\n", static_cast<unsigned long>(foc.fault.code)); }
            continue;
        }
        for(unsigned i = 0; i < 2; ++i)
            if(mode == IDLE || i != selected)
                motor::set_target_current(static_cast<motor::motor_id>(i), {});
        if(mode == IDLE) continue;
        const auto &enc = selected == 0 ? sensors.left_encoder : sensors.right_encoder;
        const auto &feedback = foc.current[selected];
        const int64_t now = esp_timer_get_time();
        if(enc.timestamp_us == last_encoder) continue;
        last_encoder = enc.timestamp_us;
        const float dt = previous ? (now-previous)*1e-6f : 0.001f; previous = now;
        const float speed = feedback.encoder_direction * enc.speed_rad_s;
        float error, proportional;
        if(mode == SCAN) {
            trajectory += clamp(reference - trajectory, 0.8f * dt);
            error = feedback.encoder_direction * (trajectory - enc.angle_rad);
            servo_speed = (0.0005f * servo_speed + feedback.encoder_direction *
                           (enc.angle_rad-servo_angle)) / (0.0005f+dt);
            servo_angle = enc.angle_rad;
            proportional = position_kp * error - position_kd * servo_speed;
        }
        else {
            const float ramp = fminf((now-began)*1e-6f/0.7f, 1.0f);
            error = speed_ref*ramp - speed; proportional = 0.02f * error;
        }
        const float candidate = integral + (mode == SCAN ? position_ki : 0.08f)*dt*error;
        const float output = proportional + candidate;
        if(fabsf(output) <= cap || output*error < 0) integral = candidate;
        motor::target_current target{}; target.iq_A = clamp(output, cap);
        motor::set_target_current(static_cast<motor::motor_id>(selected), target);
        ++ticks;
        if(mode == SPEED) {
            if(ticks % 5 == 0 && count < 4000)
                records[count++] = {(now-began)*1e-6f, enc.single_turn_rad, speed,
                                    feedback.iq_A, target.iq_A, feedback.id_A};
            if((now-began)*1e-6f >= duration) { idle(); printf("RUN_END %u\n", count); }
            continue;
        }
        const float elapsed = (now-segment)*1e-6f;
        if(step == 0 && ticks % 100 == 0)
            printf("P,%.3f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n", elapsed, reference-trajectory, error, speed, feedback.iq_A, target.iq_A, integral);
        if(elapsed >= scan_seconds-0.12f && fabsf(reference-trajectory) < 0.0001f && fabsf(error) < 0.008f) {
            ++samples; sum_angle += enc.angle_rad-reference; sum_speed += speed;
            sum_iq += feedback.iq_A; sum_target += target.iq_A; sum_id += feedback.id_A;
        }
        else clear_sums();
        if(elapsed >= scan_seconds && samples >= 80 && fabsf(sum_speed/samples) < 0.08f) {
            float angle = fmodf(reference + sum_angle/samples, PI2); if(angle < 0) angle += PI2;
            printf("C,%u,%u,%d,%.7f,%.5f,%.6f,%.6f,%.6f,%u,%.6f\n", selected, step,
                   step < N ? 1 : -1, angle, sum_speed/samples, sum_iq/samples,
                   sum_target/samples, sum_id/samples, samples, error);
            ++step;
            if(step == 2*N) { idle(); puts("SCAN_END"); }
            else {
                reference = origin + (step < N ? step : 2*N-1-step)*(PI2/N);
                segment = now; clear_sums();
            }
        }
        else if(elapsed > (step == 0 ? 15.0f : 3.0f)) {
            printf("SCAN_FAIL step=%u error=%.5f speed=%.5f iq=%.5f samples=%u\n",
                   step,error,speed,feedback.iq_A,samples); idle();
        }
    }
}
}
bool init() {
    return xTaskCreatePinnedToCore(task, "anticogging", 6144, nullptr, 2, nullptr, 0) == pdPASS;
}
}






