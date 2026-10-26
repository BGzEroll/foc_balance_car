#pragma once
#include <stdint.h>
namespace pid_config {
constexpr float DEFAULT_GAINS[10]={9.500000000e-01f,1.000000000e-01f,2.100000000e+00f,2.000000000e+01f,0.000000000e+00f,9.000000000e+00f,6.000000000e-01f,2.000000000e+00f,2.200000000e+00f,-1.564208089e-03f};
constexpr float SPEED_LIMIT=.35f;
constexpr float PITCH_TARGET_LIMIT=.2617993878f;
constexpr float VELOCITY_INTEGRAL_LIMIT=.5f;
constexpr float PITCH_INTEGRAL_LIMIT=.25f;
constexpr float YAW_INTEGRAL_LIMIT=.5f;
constexpr uint32_t REVISION=406002573U;
constexpr const char *DESIGN_SHA256="18331b8d16e76252b26f500f1bea07c26af8ef0befba049f730d6dbcc4c832a0";
}
