#ifndef FOC_MATH_H
#define FOC_MATH_H

#include <math.h>
#include <stdint.h>
#include "esp_attr.h"

namespace foc_math
{
    constexpr float TWO_PI = 6.28318530718f;
    constexpr uint32_t TABLE_SIZE = 4096;
    constexpr uint32_t TABLE_MASK = TABLE_SIZE - 1;
    static DRAM_ATTR float sine_table[TABLE_SIZE];

    inline void init()
    {
        for(uint32_t i = 0; i < TABLE_SIZE; ++i)
        {
            sine_table[i] = sinf(static_cast<float>(i) * (TWO_PI / TABLE_SIZE));
        }
    }

    // 单圈角度产生的正常输入在此范围内；保留大角度归一化能力。
    inline float wrap(float angle)
    {
        if(fabsf(angle) > 64.0f)
        {
            angle = fmodf(angle, TWO_PI);
        }
        else
        {
            angle -= static_cast<int32_t>(angle * (1.0f / TWO_PI)) * TWO_PI;
        }

        if(angle < 0.0f)
        {
            angle += TWO_PI;
        }

        return angle >= TWO_PI ? 0.0f : angle;
    }

    // 输入为 [0, 2π)，余弦共用正弦表的四分之一周期偏移。
    inline void sincos(float angle, float &sn, float &cs)
    {
        const float position = angle * (TABLE_SIZE / TWO_PI);
        const uint32_t whole = static_cast<uint32_t>(position);
        const float fraction = position - static_cast<float>(whole);
        const uint32_t index = whole & TABLE_MASK;
        const uint32_t cosine = (index + TABLE_SIZE / 4) & TABLE_MASK;
        sn = sine_table[index] + fraction *
            (sine_table[(index + 1) & TABLE_MASK] - sine_table[index]);
        cs = sine_table[cosine] + fraction *
            (sine_table[(cosine + 1) & TABLE_MASK] - sine_table[cosine]);
    }
}

#endif
