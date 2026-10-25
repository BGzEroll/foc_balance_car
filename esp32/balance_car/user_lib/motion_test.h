#pragma once
#include <cmath>
#include <cstdint>

namespace motion_test {
// Scoped to an explicitly started test; normal BALANCE has no distance guard.
struct state {
    bool armed=false, aborted=false, hard_stop=false;
    uint64_t began_us=0;
    float origin=0, position=0, speed=0, peak=0, amplitude=.05f;
    unsigned phase=0;
    void start(uint64_t now,float x,float excursion=.05f) { *this=state{};armed=true;began_us=now;origin=position=x;amplitude=excursion; }
    void advance(uint64_t now,float x) {
        if(!armed)return;
        const float displacement=std::fabs(x-origin);
        peak=std::fmax(peak,displacement);
        if(!std::isfinite(x) || displacement>=.19f)hard_stop=true;
        if(displacement>=.17f || hard_stop)aborted=true;
        if(aborted){position=origin;speed=0;phase=5;return;}
        const double seconds=static_cast<double>(now-began_us)*1e-6;
        if(seconds>=48){position=origin;speed=0;phase=4;return;}
        phase=static_cast<unsigned>(seconds/12);
        const float from[4]={0,amplitude,0,-amplitude},to[4]={amplitude,0,-amplitude,0};
        const double u=std::fmin(1.0,(seconds-12*phase)/6);
        const double u2=u*u,u3=u2*u;
        const double h=u3*(10+u*(-15+6*u));
        const double dh=30*u2*(1-u)*(1-u)/6;
        position=origin+from[phase]+(to[phase]-from[phase])*h;
        speed=(to[phase]-from[phase])*dh;
    }
};
}
