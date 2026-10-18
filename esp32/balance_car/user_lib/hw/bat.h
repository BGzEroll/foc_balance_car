#ifndef BAT_H
#define BAT_H

namespace bat
{
    struct state
    {
        bool valid = false;     // ADC 采样和校准换算成功，不表示电池状态正常

        float voltage_V = 0.0f;     // 开关后的 PVCC 母线电压，单位 V
    };

    bool init();
    bool get_package(state &out);
}

#endif
