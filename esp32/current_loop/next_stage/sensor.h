#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>

namespace sensor
{
    struct encoder_data
    {
        uint64_t timestamp_us = 0;

        float angle_rad = 0.0f;
        float speed_rad_s = 0.0f;
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
    void begin_inline();
    // Recover a stalled asynchronous bus only while the caller has stopped PWM.
    void poll_inline(bool allow_recovery = true);
    struct health_data {
        uint32_t left_updates, right_updates;
        uint32_t left_recoveries, right_recoveries;
        uint32_t left_submit_errors, right_submit_errors;
        uint32_t left_rejected_events, right_rejected_events;
    };
    health_data get_health();
    // Test-branch diagnostic: one address NACK, consumed only while stopped.
    void request_test_nack();
    bool get_package(package &snapshot);
}

#endif
