
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
    };

    struct foc_info
    {
        bool initialized = false;
        bool ready = false;

        target_current target[2] = {};
        current_info current[2] = {};

        float bus_voltage_V = 0.0f;
        uint64_t bus_timestamp_us = 0;

        fault_info fault = {};
    };

    bool init();
    bool set_target_current(motor_id id, const target_current &target);
    bool get_info(foc_info &snapshot);
    bool set_cogging_point(uint32_t motor, uint32_t point, float current_A);
    void enable_cogging(bool enabled);
    void stop();
}

#endif
