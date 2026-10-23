#pragma once
#include "sdkconfig.h"

namespace realtime_config {
constexpr bool WIRELESS_DEBUG_ENABLED = false;
constexpr unsigned SENSOR_FOC_PRIORITY = 5;
constexpr unsigned UPPER_PRIORITY = 2;
constexpr unsigned NETWORK_PRIORITY = 1;
constexpr unsigned SENSOR_FOC_CORE = 1;
constexpr unsigned NETWORK_CORE = 0;
static_assert(NETWORK_PRIORITY < UPPER_PRIORITY);
static_assert(UPPER_PRIORITY < SENSOR_FOC_PRIORITY);
static_assert(CONFIG_LWIP_TCPIP_TASK_PRIO < SENSOR_FOC_PRIORITY);
static_assert(CONFIG_LWIP_TCPIP_TASK_PRIO < UPPER_PRIORITY);
static_assert(CONFIG_LWIP_TCPIP_TASK_AFFINITY == 0);
}
