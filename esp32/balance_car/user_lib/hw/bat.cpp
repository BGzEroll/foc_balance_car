#include "bat.h"
#include <stdint.h>

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

namespace bat
{
    namespace
    {
        constexpr adc_channel_t CHANNEL = ADC_CHANNEL_0;
        constexpr adc_atten_t ATTEN = ADC_ATTEN_DB_12;
        constexpr adc_bitwidth_t BITWIDTH = ADC_BITWIDTH_12;
        constexpr uint32_t SAMPLE_COUNT = 16;

        // PVCC 经 R9/R13 分压接 GPIO1，测量开关后的母线电压。
        // R9 20kΩ，R13 10kΩ。
        constexpr float DIVIDER_RATIO = 3.0f;

        adc_oneshot_unit_handle_t adc = nullptr;
        adc_cali_handle_t calibration = nullptr;
    }

    /**
     * @brief 初始化母线电压 ADC 及芯片曲线校准
     *
     * @return true 初始化成功或已经初始化
     * @return false ADC 单元创建、通道配置或芯片曲线校准初始化失败
     */
    bool init()
    {
        if(adc != nullptr){return true;}

        adc_oneshot_unit_init_cfg_t unit_config{};
        unit_config.unit_id = ADC_UNIT_1;

        esp_err_t error = adc_oneshot_new_unit(&unit_config, &adc);
        if(error != ESP_OK)
        {
            ESP_LOGE("bat", "ADC 初始化失败：%s", esp_err_to_name(error));
            return false;
        }

        adc_oneshot_chan_cfg_t channel_config{};
        channel_config.atten = ATTEN;
        channel_config.bitwidth = BITWIDTH;
        error = adc_oneshot_config_channel(adc, CHANNEL, &channel_config);

        if(error == ESP_OK)
        {
            adc_cali_curve_fitting_config_t calibration_config{};
            calibration_config.unit_id = ADC_UNIT_1;
            calibration_config.chan = CHANNEL;
            calibration_config.atten = ATTEN;
            calibration_config.bitwidth = BITWIDTH;
            error = adc_cali_create_scheme_curve_fitting(&calibration_config, &calibration);
        }

        if(error != ESP_OK)
        {
            ESP_LOGE("bat", "ADC 通道配置或校准失败：%s", esp_err_to_name(error));
            adc_oneshot_del_unit(adc);
            adc = nullptr;
            return false;
        }

        return true;
    }

    /**
     * @brief 采样并获取母线电压数据
     *
     * @param[out] out 母线电压数据，电压单位为 V；失败时清空并标记无效
     *
     * @return true 采样与校准换算成功
     * @return false 未初始化、ADC 采样失败、ADC 饱和或校准换算失败
     */
    bool get_package(state &out)
    {
        out = state{};
        if(adc == nullptr){return false;}

        int32_t voltage_sum_mv = 0;
        for(uint32_t i = 0; i < SAMPLE_COUNT; i++)
        {
            int raw = 0;
            int voltage_mv = 0;
            if(adc_oneshot_read(adc, CHANNEL, &raw) != ESP_OK || raw >= 4095)
            {
                return false;
            }
            if(adc_cali_raw_to_voltage(calibration, raw, &voltage_mv) != ESP_OK)
            {
                return false;
            }
            voltage_sum_mv += voltage_mv;
        }

        out.voltage_V = static_cast<float>(voltage_sum_mv) /
                           static_cast<float>(SAMPLE_COUNT) * 0.001f * DIVIDER_RATIO;
        out.valid = true;
        return true;
    }
}
