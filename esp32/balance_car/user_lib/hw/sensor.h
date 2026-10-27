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
    };

    struct imu_data
    {
        uint64_t timestamp_us = 0;
        int16_t raw_acc[3] = {};
        int16_t raw_gyro[3] = {};

        float temperature = 0.0f;

        float acc[3] = {};
        float gyro[3] = {};
        float angle[3] = {};
        float gravity[3] = {};
        float gravity_norm_g = 0.0f;
    };

    struct package
    {
        encoder_data left_encoder;
        encoder_data right_encoder;
        imu_data imu;
    };

    bool init();
    void poll(uint32_t notifications, bool allow_recovery = true);
    bool wait_for_imu();
    bool get_package(package &snapshot);
    bool update_attitude(
        package &snapshot,
        const float wheel_angle_rad[2],
        bool grounded,
        bool allow_yaw_learning);
    void reset_attitude();
}

#endif
