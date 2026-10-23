
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

namespace motor
{
    enum class motor_id : uint8_t
    {
        LEFT = 0,
        RIGHT = 1
    };

    enum class fault_code : uint32_t
    {
        NONE = 0,
        SENSOR_INIT,
        SENSOR_TIMEOUT,
        BUS_VOLTAGE,
        BUS_STALE,
        ADC_READ,
        OVERCURRENT,
        SAMPLE_TIMEOUT,
        PWM_UPDATE,
        ALIGNMENT,
        TASK_TIMEOUT
    };

    struct target_current
    {
        float id_A = 0.0f;
        float iq_A = 0.0f;
    };

    struct current_info
    {
        uint64_t timestamp_us = 0;

        float electrical_angle_rad = 0.0f;
        float mechanical_angle_rad = 0.0f;
        float mechanical_speed_rad_s = 0.0f;
        float encoder_direction = 0.0f;

        float phase_A[3] = {};
        float id_A = 0.0f;
        float iq_A = 0.0f;

        bool valid = false;
    };

    struct fault_info
    {
        bool active = false;
        fault_code code = fault_code::NONE;

        uint8_t motor_index = 0xFF;
        uint64_t timestamp_us = 0;

        uint16_t raw_a = 0;
        uint16_t raw_b = 0;
        uint32_t detail = 0; // high byte: timeout source; low 24 bits: age/dt in us
        uint32_t adc_age_us = 0;
        uint32_t heartbeat_age_us = 0;
        uint32_t control_phase = 0;
        uint32_t sample_flags = 0, sample_start_tick = 0, sample_end_tick = 0;
        uint32_t sample_duty_a = 0, sample_duty_b = 0;
    };

    struct foc_info
    {
        bool initialized = false;
        bool ready = false;
        bool calibrated = false;
        bool running = false;
        uint32_t pwm_core = 0;

        target_current target[2] = {};
        current_info current[2] = {};

        float bus_voltage_V = 0.0f;
        uint64_t bus_timestamp_us = 0;

        fault_info fault = {};
    };

    bool init();
    bool calibrate();
    bool start(uint32_t duration_ms, uint32_t mask = 3);
    void keep_alive();
    void stop();
    bool set_target_current(motor_id id, const target_current &target);
    bool get_info(foc_info &snapshot);
    // OTA interlock is acknowledged by the motor task before any flash writes.
    void set_update_lock(bool locked);
    bool update_locked();
    bool update_stopped();
}

#endif
