#pragma once

#include "cc1101_manager.h"
#include "lvgl.h"
#include <stdint.h>

typedef void (*UiThresholdChangedCb)(int value);
typedef void (*UiThresholdSavedCb)(int value);
typedef void (*UiSplashDoneCb)();

void ui_manager_init(int initial_threshold,
                     UiThresholdChangedCb on_threshold_changed,
                     UiThresholdSavedCb on_threshold_saved);
void ui_manager_create_splash(UiSplashDoneCb on_splash_done);

void ui_manager_queue_update(float freq_mhz, int rssi, const char *modulation, const char *status);
void ui_manager_set_last_signal(float freq_mhz, int rssi, const char *modulation);
void ui_manager_queue_spectrum_update(const Cc1101SweepResult &sweep);
void ui_manager_queue_battery_update(uint8_t battery_state, float battery_voltage);
void ui_manager_process_pending_update();
bool ui_manager_is_spectrum_active();
bool ui_manager_is_subghz_active();
bool ui_manager_is_freq_only_active();

int ui_manager_get_threshold();
