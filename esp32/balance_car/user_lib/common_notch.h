#pragma once

// MATLAB-screened 200Hz common-current notch: 12.5Hz, pole radius .92, unity DC.
namespace common_notch
{
constexpr float B0=0.96203862779054072f;
constexpr float B1=-1.7776155954018491f;
constexpr float B2=0.96203862779054072f;
constexpr float A1=-1.6999383398207677f;
constexpr float A2=0.8464f;
struct state
{
    float x1=0,x2=0,y1=0,y2=0;
    bool initialized=false;
    float update(float input)
    {
        if(!initialized)
        {
            x1=x2=y1=y2=input;initialized=true;
            return input;
        }
        const float output=B0*input+B1*x1+B2*x2-A1*y1-A2*y2;
        x2=x1;x1=input;y2=y1;y1=output;
        return output;
    }
};
}
