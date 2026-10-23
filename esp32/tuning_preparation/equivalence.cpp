#include "balance.h"
#include "old_balance.h"
#include <cmath>
#include <cstdio>
#include <algorithm>
int main() {
    constexpr float kt=.013311959501823515f;
    double max_error=0; unsigned samples=0;
    for(int group=0;group<6;++group) {
        old_balance::state old_state{};balance::state state{};
        old_balance::gains old_g{};
        if(group%3==1) old_g={.7f,.3f,.04f};
        if(group%3==2) old_g={.5f,.2f,.05f};
        balance::gains g{old_g.velocity_kp,old_g.velocity_ki,old_g.pitch_kd/kt};
        for(int k=0;k<6000;++k) {
            float t=k*.005f,scale=(k<1000?.002f:k<2000?.02f:1.f);
            balance::observation m{scale*.2f*std::sin(.9f*t),scale*.1f*std::sin(1.7f*t),
                scale*.17f*std::cos(1.7f*t),scale*3.4f*std::sin(.7f*t),scale*2.38f*std::cos(.7f*t)};
            balance::reference ref{0,0,0,0};balance::limits lim{};lim.current_A=group<3?.7f:.25f;
            old_balance::observation om{m.speed_m_s,m.pitch_rad,m.pitch_rate_rad_s,m.yaw_rad,m.yaw_rate_rad_s};
            old_balance::reference ot{};old_balance::limits ol{};ol.current_A=lim.current_A;
            auto a=old_balance::update(om,ot,ol,old_state,old_g);auto b=balance::update(m,ref,lim,state,g);
            const double err=std::max({std::abs(double(a.current_A[0]-b.current_A[0])),
                std::abs(double(a.current_A[1]-b.current_A[1])),std::abs(double(a.target_pitch_rad-b.target_pitch_rad)),
                std::abs(double(old_state.distance_m-state.distance_m))});
            max_error=std::max(max_error,err);++samples;
            std::printf("%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n",
                group,m.speed_m_s,m.pitch_rad,m.pitch_rate_rad_s,m.yaw_rad,m.yaw_rate_rad_s,
                lim.current_A,g.velocity_kp,g.velocity_ki,g.pitch_kd,b.current_A[0],b.current_A[1],b.target_pitch_rad);
        }
    }
    std::fprintf(stderr,"{\"samples\":%u,\"max_current_or_state_error\":%.12g}\n",samples,max_error);
    return max_error<1e-4 ? 0:1;
}
