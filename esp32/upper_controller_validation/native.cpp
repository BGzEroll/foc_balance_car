#include "balance.h"

#include <cstdio>

int main(int argc, char **)
{
    if(argc > 1)
    {
        float angle[3], gyro[3], wheel[2];

        while(std::scanf("%f,%f,%f,%f,%f,%f,%f,%f\n", &angle[0], &angle[1], &angle[2],
                         &gyro[0], &gyro[1], &gyro[2], &wheel[0], &wheel[1]) == 8)
        {
            const balance::observation measured = balance::observe(angle, gyro, wheel);
            std::printf("%.9g,%.9g,%.9g,%.9g,%.9g\n", measured.speed_m_s, measured.pitch_rad,
                        measured.pitch_rate_rad_s, measured.yaw_rad, measured.yaw_rate_rad_s);
        }
        return 0;
    }

    int group = 0;
    int previous = -1;
    balance::state state{};
    balance::observation measured{};
    balance::reference target{};
    balance::limits limit{};

    while(std::scanf("%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", &group,
          &measured.speed_m_s, &measured.pitch_rad, &measured.pitch_rate_rad_s,
          &measured.yaw_rad, &measured.yaw_rate_rad_s,
          &target.position_m, &target.speed_m_s, &target.yaw_rad,
          &target.yaw_rate_rad_s, &limit.current_A) == 11)
    {
        if(group != previous)
        {
            state = balance::state{};
            previous = group;
        }

        const balance::output output = balance::update(measured, target, limit, state);
        std::printf("%.9g,%.9g,%.9g,%.9g,%.9g\n", output.current_A[0], output.current_A[1],
                    output.target_speed_m_s, output.target_pitch_rad, state.distance_m);
    }
}
