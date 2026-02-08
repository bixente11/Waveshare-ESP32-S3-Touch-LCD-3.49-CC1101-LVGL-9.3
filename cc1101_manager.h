#pragma once

#include <stddef.h>
#include <stdint.h>

struct Cc1101ScanResult {
    bool signal_detected;
    float detected_freq_mhz;
    int detected_rssi_dbm;
    bool is_fsk;
    int best_rssi_dbm;
    int scan_count;
};

constexpr size_t CC1101_SWEEP_MAX_SAMPLES = 128;

struct Cc1101SweepResult {
    bool valid;
    float start_freq_mhz;
    float end_freq_mhz;
    uint16_t sample_count;
    int16_t rssi_dbm[CC1101_SWEEP_MAX_SAMPLES];
    float max_freq_mhz;
    int max_rssi_dbm;
};

bool cc1101_manager_init(int rssi_threshold);
Cc1101ScanResult cc1101_manager_scan_once(int rssi_threshold);
bool cc1101_manager_capture_sweep(float start_freq_mhz,
                                  float end_freq_mhz,
                                  uint16_t sample_count,
                                  Cc1101SweepResult *out_result);
void cc1101_manager_restore_scan_mode();
