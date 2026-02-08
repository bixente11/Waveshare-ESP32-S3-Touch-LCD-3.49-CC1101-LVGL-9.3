#include "cc1101_manager.h"

#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>

namespace {

// CC1101 wiring
constexpr int CC1101_CS = 3;
constexpr int CC1101_GDO0 = 5;
constexpr int CC1101_MOSI = 39;
constexpr int CC1101_MISO = 40;
constexpr int CC1101_SCK = 41;

struct FrequencyRSSI {
    uint32_t frequency_coarse;
    int rssi_coarse;
    uint32_t frequency_fine;
    int rssi_fine;
    bool is_fsk;
};

static const uint32_t kSubGHzFrequencyList[] = {
    300000000, 302757000, 303875000, 303900000, 304250000,
    307000000, 307500000, 307800000, 309000000, 310000000,
    312000000, 312100000, 312200000, 313000000, 313850000,
    314000000, 314350000, 314980000, 315000000, 318000000,
    330000000, 345000000, 348000000, 350000000, 387000000,
    390000000, 418000000, 430000000, 430500000, 431000000,
    431500000, 433075000, 433220000, 433420000, 433657070,
    433889000, 433920000, 434075000, 434176948, 434190000,
    434390000, 434420000, 434620000, 434775000, 438900000,
    440175000, 464000000, 467750000, 779000000, 868350000,
    868400000, 868800000, 868950000, 906400000, 915000000,
    925000000, 928000000
};

SPIClass spiCC1101(FSPI);
CC1101 cc1101 = new Module(CC1101_CS, CC1101_GDO0, RADIOLIB_NC, RADIOLIB_NC, spiCC1101);
int g_scan_count = 0;
// Set when spectrum mode was active and scan profile must be fully restored.
bool g_need_scan_reinit = false;

float clamp_freq(float value_mhz, float min_mhz, float max_mhz) {
    if (value_mhz < min_mhz) {
        return min_mhz;
    }
    if (value_mhz > max_mhz) {
        return max_mhz;
    }
    return value_mhz;
}

void apply_scan_profile() {
    // Wide bandwidth scan profile for fast coarse detection.
    cc1101.standby();
    cc1101.setOOK(false);
    cc1101.setRxBandwidth(650);
    cc1101.setFrequencyDeviation(47.6);
    cc1101.setFrequency(433.92);
}

void apply_sweep_profile() {
    // Narrower profile used for spectrum bars around one band.
    cc1101.standby();
    cc1101.setOOK(false);
    cc1101.setRxBandwidth(200);
}

bool reinit_for_scan() {
    // Hard reinit protects against radio state corruption after spectrum sweeps.
    spiCC1101.begin(CC1101_SCK, CC1101_MISO, CC1101_MOSI, CC1101_CS);
    const int state = cc1101.begin();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[CC1101] scan reinit failed: %d\n", state);
        return false;
    }
    apply_scan_profile();
    g_need_scan_reinit = false;
    Serial.println("[CC1101] scan reinit OK");
    return true;
}

bool detect_modulation(float frequency_mhz) {
    int rssi_ask = -120;
    int rssi_fsk = -120;

    const float bandwidth = (frequency_mhz > 850.0f) ? 250.0f : 200.0f;
    const float deviation = (frequency_mhz > 850.0f) ? 50.0f : 47.6f;

    // ASK/OOK measurement.
    cc1101.setOOK(true);
    cc1101.setFrequency(frequency_mhz);
    cc1101.setRxBandwidth(bandwidth);
    cc1101.standby();
    delay(2);
    cc1101.receiveDirect();
    delay(8);
    rssi_ask = cc1101.getRSSI();

    // FSK measurement.
    cc1101.setOOK(false);
    cc1101.setFrequency(frequency_mhz);
    cc1101.setRxBandwidth(bandwidth);
    cc1101.setFrequencyDeviation(deviation);
    cc1101.standby();
    delay(2);
    cc1101.receiveDirect();
    delay(8);
    rssi_fsk = cc1101.getRSSI();

    Serial.printf("    [Modulation] ASK RSSI: %d dBm | FSK RSSI: %d dBm\n", rssi_ask, rssi_fsk);
    return (rssi_fsk > rssi_ask);
}

}  // namespace

bool cc1101_manager_init(int rssi_threshold) {
    Serial.println("\n[CC1101] Initialisation...");

    spiCC1101.begin(CC1101_SCK, CC1101_MISO, CC1101_MOSI, CC1101_CS);

    const int state = cc1101.begin();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.print("[CC1101] ERREUR d'initialisation: ");
        Serial.println(state);
        return false;
    }

    cc1101.setFrequency(433.92);
    g_need_scan_reinit = false;

    Serial.println("[CC1101] ✓ Initialise avec succes");
    Serial.printf("[CONFIG] Seuil RSSI: %d dBm\n", rssi_threshold);
    Serial.printf("[CONFIG] Nombre de frequences: %d\n",
                  static_cast<int>(sizeof(kSubGHzFrequencyList) / sizeof(kSubGHzFrequencyList[0])));
    return true;
}

Cc1101ScanResult cc1101_manager_scan_once(int rssi_threshold) {
    FrequencyRSSI freq_rssi = {
        .frequency_coarse = 0,
        .rssi_coarse = -100,
        .frequency_fine = 0,
        .rssi_fine = -100,
        .is_fsk = false,
    };

    g_scan_count++;

    if (g_need_scan_reinit) {
        // First scan after spectrum screen gets a full radio reset path.
        if (!reinit_for_scan()) {
            return Cc1101ScanResult{};
        }
    } else {
        apply_scan_profile();
    }

    // Coarse scan over known sub-GHz channels.
    for (size_t i = 0; i < sizeof(kSubGHzFrequencyList) / sizeof(kSubGHzFrequencyList[0]); i++) {
        const uint32_t freq = kSubGHzFrequencyList[i];
        cc1101.setFrequency(freq / 1e6);
        cc1101.receiveDirect();
        delay(3);
        const int rssi = cc1101.getRSSI();

        if (rssi > freq_rssi.rssi_coarse) {
            freq_rssi.rssi_coarse = rssi;
            freq_rssi.frequency_coarse = freq;
        }
    }

    Cc1101ScanResult result{};
    result.best_rssi_dbm = freq_rssi.rssi_coarse;
    result.scan_count = g_scan_count;

    if (freq_rssi.rssi_coarse <= rssi_threshold) {
        return result;
    }

    Serial.println("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    Serial.printf("🔍 Signal detecte (scan #%d)\n", g_scan_count);
    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    Serial.printf("  [Scan grossier] Frequence: %.2f MHz | RSSI: %d dBm\n",
                  freq_rssi.frequency_coarse / 1e6, freq_rssi.rssi_coarse);

    cc1101.setRxBandwidth(58);
    Serial.println("  [Scan fin] Affinement en cours...");

    // Fine scan around the best coarse hit.
    for (uint32_t f = freq_rssi.frequency_coarse - 300000;
         f <= freq_rssi.frequency_coarse + 300000;
         f += 20000) {
        cc1101.setFrequency(f / 1e6);
        cc1101.receiveDirect();
        delay(3);
        const int rssi = cc1101.getRSSI();

        if (rssi > freq_rssi.rssi_fine) {
            freq_rssi.rssi_fine = rssi;
            freq_rssi.frequency_fine = f;
        }
    }

    Serial.printf("  [Scan fin] Frequence affinee: %.2f MHz | RSSI: %d dBm\n",
                  freq_rssi.frequency_fine / 1e6, freq_rssi.rssi_fine);

    Serial.println("  [Detection] Analyse de la modulation...");
    freq_rssi.is_fsk = detect_modulation(freq_rssi.frequency_fine / 1e6);

    Serial.println("\n  ╔════════════════════════════════════╗");
    Serial.printf("  ║  🎯 SIGNAL DETECTE                 ║\n");
    Serial.println("  ╠════════════════════════════════════╣");
    Serial.printf("  ║  Frequence: %.2f MHz          ║\n", freq_rssi.frequency_fine / 1e6);
    Serial.printf("  ║  RSSI:      %d dBm               ║\n", freq_rssi.rssi_fine);
    Serial.printf("  ║  Modulation: %-18s ║\n", freq_rssi.is_fsk ? "FSK" : "ASK/OOK");
    Serial.println("  ╚════════════════════════════════════╝");
    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");

    result.signal_detected = true;
    result.detected_freq_mhz = freq_rssi.frequency_fine / 1e6;
    result.detected_rssi_dbm = freq_rssi.rssi_fine;
    result.is_fsk = freq_rssi.is_fsk;
    return result;
}

bool cc1101_manager_capture_sweep(float start_freq_mhz,
                                  float end_freq_mhz,
                                  uint16_t sample_count,
                                  Cc1101SweepResult *out_result) {
    if (!out_result) {
        return false;
    }

    Cc1101SweepResult result{};
    result.valid = false;
    result.max_rssi_dbm = -120;
    result.max_freq_mhz = 0.0f;

    if (sample_count < 2 || sample_count > CC1101_SWEEP_MAX_SAMPLES) {
        *out_result = result;
        apply_scan_profile();
        return false;
    }

    constexpr float kMinFreqMhz = 300.0f;
    constexpr float kMaxFreqMhz = 928.0f;

    float start = clamp_freq(start_freq_mhz, kMinFreqMhz, kMaxFreqMhz);
    float end = clamp_freq(end_freq_mhz, kMinFreqMhz, kMaxFreqMhz);
    if (end <= start) {
        end = start + 0.1f;
    }

    result.start_freq_mhz = start;
    result.end_freq_mhz = end;
    result.sample_count = sample_count;

    // Uniform sweep over the requested band.
    const float freq_step = (end - start) / static_cast<float>(sample_count - 1);

    apply_sweep_profile();

    for (uint16_t i = 0; i < sample_count; ++i) {
        const float freq_mhz = start + freq_step * static_cast<float>(i);
        cc1101.setFrequency(freq_mhz);
        cc1101.receiveDirect();
        delayMicroseconds(2500);

        const int rssi = cc1101.getRSSI();
        result.rssi_dbm[i] = static_cast<int16_t>(rssi);

        if (rssi > result.max_rssi_dbm) {
            result.max_rssi_dbm = rssi;
            result.max_freq_mhz = freq_mhz;
        }
    }

    apply_scan_profile();
    result.valid = true;
    *out_result = result;
    return true;
}

void cc1101_manager_restore_scan_mode() {
    // Defer full reinit to next scan_once call.
    g_need_scan_reinit = true;
}
