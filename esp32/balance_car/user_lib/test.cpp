#include "test.h"
#include "hw/sensor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace test
{
    static void task(void *)
    {
        while(true)
        {
            sensor::package p{};
            if(sensor::get_package(p))
            {
                ESP_LOGI("sensor", "M1 t=%llu angle_rad=%.6f speed_rad_s=%.6f | M2 t=%llu angle_rad=%.6f speed_rad_s=%.6f | IMU t=%llu acc=%.4f,%.4f,%.4f gyro=%.5f,%.5f,%.5f angle=%.4f,%.4f,%.4f temp=%.2f",
                    (unsigned long long)p.left_encoder.timestamp_us,
                    p.left_encoder.angle_rad, p.left_encoder.speed_rad_s,
                    (unsigned long long)p.right_encoder.timestamp_us,
                    p.right_encoder.angle_rad, p.right_encoder.speed_rad_s,
                    (unsigned long long)p.imu.timestamp_us,
                    p.imu.acc[0],p.imu.acc[1],p.imu.acc[2],
                    p.imu.gyro[0],p.imu.gyro[1],p.imu.gyro[2],
                    p.imu.angle[0],p.imu.angle[1],p.imu.angle[2],p.imu.temperature);
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    bool init()
    {
        return xTaskCreate(task, "sensor_test", 4096, nullptr, 1, nullptr) == pdPASS;
    }
}
