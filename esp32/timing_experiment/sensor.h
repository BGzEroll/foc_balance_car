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
    void poll_inline();
    bool get_package(package &snapshot);
}

#endif
