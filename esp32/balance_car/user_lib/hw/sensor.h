#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>

namespace sensor
{
    struct encoder_data
    {
        uint64_t timestamp_us = 0;

        float angle_rad = 0.0f;
        float single_turn_rad = 0.0f;
        float speed_rad_s = 0.0f;
        float observer_speed_rad_s = 0.0f;
    };

    struct imu_data
    {
        uint64_t timestamp_us = 0;

        float temperature = 0.0f;

        float acc[3] = {};
        float gyro[3] = {};
        float angle[3] = {};
    };

    struct package
    {
        encoder_data left_encoder;
        encoder_data right_encoder;
        imu_data imu;
    };

    bool init();
    void poll(uint32_t notifications, bool allow_recovery = true);
    bool get_package(package &snapshot);

    struct encoder_record {
        uint32_t time_us;
        uint16_t motor_raw;
        int16_t delta_counts;
        float lpf_speed, ab_speed;
    };
    struct __attribute__((packed)) encoder_capture_header {
        uint32_t magic, count, tau_us, enabled;
        uint64_t first_us;
    };
    bool begin_encoder_capture();
    const encoder_record *get_encoder_capture(encoder_capture_header &header);
    void set_encoder_observer(uint32_t tau_us);
    uint32_t encoder_observer_us();
}

#endif
