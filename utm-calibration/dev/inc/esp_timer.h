#pragma once
#include <cstdint>
double sim_now_s();
inline int64_t esp_timer_get_time() { return (int64_t)(sim_now_s() * 1e6); }
