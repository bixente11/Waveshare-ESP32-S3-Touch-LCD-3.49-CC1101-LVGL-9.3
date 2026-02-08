#pragma once

#include <stdint.h>

typedef void (*BatteryUpdateCb)(uint8_t battery_state, float battery_voltage);

void battery_manager_init(BatteryUpdateCb on_battery_update);
