#include "test.h"

#include "hw/motor.h"
#include "hw/sensor.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>

namespace test
{
    namespace
    {
        constexpr uint32_t DISPLAY_PERIOD_MS = 200;

        /**
         * @brief 显示传感器与电流环快照，不下发目标电流
         */
        void display_task(void *)
        {
            while(true)
            {
                sensor::package sensors{};
                const bool sensor_ready = sensor::get_package(sensors);

                motor::foc_info foc{};
                motor::get_info(foc);

                printf(
                    "ready=%u imu=%u bus=%.3f V fault=%lu motor=%u\n",
                    static_cast<unsigned>(foc.ready),
                    static_cast<unsigned>(sensor_ready),
                    foc.bus_voltage_V,
                    static_cast<unsigned long>(foc.fault.code),
                    static_cast<unsigned>(foc.fault.motor_index));

                for(uint32_t i = 0; i < 2; ++i)
                {
                    const motor::current_info &current = foc.current[i];

                    printf(
                        "M%u valid=%u speed=%+.3f rad/s "
                        "Id=%+.3f/%+.3f A Iq=%+.3f/%+.3f A\n",
                        static_cast<unsigned>(i + 1),
                        static_cast<unsigned>(current.valid),
                        current.mechanical_speed_rad_s,
                        current.id_A,
                        foc.target[i].id_A,
                        current.iq_A,
                        foc.target[i].iq_A);
                }

                printf("\n");

                vTaskDelay(pdMS_TO_TICKS(DISPLAY_PERIOD_MS));
            }
        }
    }

    /**
     * @brief 启动快照显示任务，采样与电流环由 motor 模块负责
     *
     * @return true 任务创建成功
     * @return false 任务创建失败
     */
    bool init()
    {
        return xTaskCreatePinnedToCore(
            display_task, "foc_test", 4096,
            nullptr, 1, nullptr, 0) == pdPASS;
    }
}
