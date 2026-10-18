#ifndef BAT_H
#define BAT_H

namespace bat
{
    struct state
    {
        bool valid = false; // ADC 采样和校准换算成功，不表示电池状态正常

        float voltage_V = 0.0f; // 开关后的 PVCC 母线电压，单位 V
    };

    bool init();
    // 在任务中按需采样，调用期间完成 16 次读取；不在中断或高速控制环中调用。
    state get_package();
}

#endif
