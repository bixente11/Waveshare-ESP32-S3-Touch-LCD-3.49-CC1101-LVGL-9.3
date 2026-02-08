#include "battery_manager.h"

#include "adc_bsp.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace {

constexpr uint32_t BATTERY_START_DELAY_MS = 5000;
constexpr uint32_t BATTERY_POLL_MS = 10000;

BatteryUpdateCb battery_update_cb = nullptr;
TaskHandle_t battery_task_handle = nullptr;
bool battery_manager_ready = false;

uint8_t battery_state_from_voltage(float voltage) {
    if (voltage >= 4.10f) {
        return 4;
    }
    if (voltage >= 3.95f) {
        return 3;
    }
    if (voltage >= 3.80f) {
        return 2;
    }
    if (voltage >= 3.60f) {
        return 1;
    }
    return 0;
}

void battery_task(void *param) {
    (void)param;

    uint8_t last_state = 255;
    vTaskDelay(pdMS_TO_TICKS(BATTERY_START_DELAY_MS));

    while (true) {
        float voltage = 0.0f;
        int raw = 0;
        adc_get_value(&voltage, &raw);

        const uint8_t state = battery_state_from_voltage(voltage);
        if (state != last_state) {
            last_state = state;
            if (battery_update_cb) {
                battery_update_cb(state, voltage);
            }
            Serial.printf("[BAT] raw=%d vbat=%.2fV state=%u\n", raw, voltage, state);
        }

        vTaskDelay(pdMS_TO_TICKS(BATTERY_POLL_MS));
    }
}

}  // namespace

void battery_manager_init(BatteryUpdateCb on_battery_update) {
    if (battery_manager_ready) {
        return;
    }

    battery_update_cb = on_battery_update;
    adc_bsp_init();

    xTaskCreatePinnedToCore(
        battery_task,
        "battery_task",
        3072,
        nullptr,
        1,
        &battery_task_handle,
        1
    );

    battery_manager_ready = true;
}
