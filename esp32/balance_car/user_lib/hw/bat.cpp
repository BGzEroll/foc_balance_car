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
        constexpr adc_channel_t CHANNEL = ADC_CHANNEL_0; // GPIO1，PCB 网络 BAT_ADC
        constexpr adc_atten_t ATTEN = ADC_ATTEN_DB_12;
        constexpr adc_bitwidth_t BITWIDTH = ADC_BITWIDTH_12;
        constexpr uint32_t SAMPLE_COUNT = 16;

        // PCB：PVCC 经 R9/R13 分压接 GPIO1，测量开关后的母线电压。
        // R9 明确为 20kΩ，R13 暂按库默认的 10kΩ，实物阻值尚未确认。
        // 原作者也注明母线 8.15V 对应 ADC 输入约 2.7V，与三倍换算一致。
        constexpr float DIVIDER_RATIO = 3.0f;

        adc_oneshot_unit_handle_t adc = nullptr;
        adc_cali_handle_t calibration = nullptr;
    }

    /**
     * @brief 初始化母线电压 ADC 及芯片曲线校准
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
     * @brief 按需采样并返回母线电压，单位 V
     */
    state get_package()
    {
        state result;
        if(adc == nullptr){return result;}

        int32_t voltage_sum_mv = 0;
        for(uint32_t i = 0; i < SAMPLE_COUNT; i++)
        {
            // SDK 的两个输出参数均为 int*，这里按驱动签名声明。
            int raw = 0;
            int voltage_mv = 0;
            if(adc_oneshot_read(adc, CHANNEL, &raw) != ESP_OK || raw >= 4095)
            {
                return result;
            }
            if(adc_cali_raw_to_voltage(calibration, raw, &voltage_mv) != ESP_OK)
            {
                return result;
            }
            voltage_sum_mv += voltage_mv;
        }

        result.voltage_V = static_cast<float>(voltage_sum_mv) /
                           static_cast<float>(SAMPLE_COUNT) * 0.001f * DIVIDER_RATIO;
        result.valid = true;
        return result;
    }
}
