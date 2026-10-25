#pragma once
#include <stdint.h>
namespace pid_config {
constexpr float DEFAULT_GAINS[10]={9.500000000e-01f,1.000000000e-01f,2.100000000e+00f,2.000000000e+01f,0.000000000e+00f,3.100000000e+00f,8.000000000e-01f,7.000000000e-01f,2.200000000e+00f,-1.564208089e-03f};
constexpr float SPEED_LIMIT=.35f;
constexpr float PITCH_TARGET_LIMIT=.2617993878f;
constexpr float VELOCITY_INTEGRAL_LIMIT=.5f;
constexpr float PITCH_INTEGRAL_LIMIT=.25f;
constexpr float YAW_INTEGRAL_LIMIT=.5f;
constexpr uint32_t REVISION=707934649U;
constexpr const char *DESIGN_SHA256="2a3239b9803e7bc9c6240f27c92e51715e20464322a6ce820f6f9b76ac4263eb";
}
