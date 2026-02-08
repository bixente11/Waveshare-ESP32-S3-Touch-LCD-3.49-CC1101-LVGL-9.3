#include "user_config.h"
#include "lvgl_port.h"
#include "cc1101_manager.h"
#include "ui_manager.h"
#include "power_manager.h"
#include "audio_feedback_manager.h"
#include "battery_manager.h"

#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "i2c_bsp.h"
#include "lcd_bl_pwm_bsp.h"

#include <Arduino.h>
#include <Preferences.h>

namespace {

// Global runtime state of the app lifecycle.
enum AppState {
    STATE_BOOT,
    STATE_SPLASH,
    STATE_SCANNING,
    STATE_POWER_OFF,
};

volatile AppState app_state = STATE_BOOT;

// Hardware and timing constants for RF/UI/power behavior.
constexpr gpio_num_t POWER_BUTTON_GPIO = GPIO_NUM_16;
constexpr gpio_num_t SYS_OUT_GPIO = GPIO_NUM_16;
constexpr gpio_num_t BOOT_BUTTON_GPIO = GPIO_NUM_0;
constexpr gpio_num_t USB_VBUS_GPIO = GPIO_NUM_4;
constexpr int SCAN_DELAY_MS = 100;
constexpr float SWEEP_START_MHZ = 433.05f;
constexpr float SWEEP_END_MHZ = 434.79f;
constexpr uint16_t SWEEP_SAMPLE_COUNT = 96;
constexpr uint32_t DETECT_BEEP_MIN_INTERVAL_MS = 900;
constexpr uint32_t POWER_EVENTS_ARM_DELAY_MS = 3000;
constexpr uint32_t POWER_EVENTS_ARM_DELAY_EXT_RESET_MS = 8000;
constexpr uint32_t POWER_OFF_GUARD_EXT_RESET_BATTERY_MS = 30000;
constexpr uint32_t STARTUP_SOUND_WAIT_MS = 3000;
constexpr uint32_t LOOP_DELAY_MS = 10;
// Hold duration required to request power off.
constexpr uint32_t POWER_HOLD_MS = 500;
constexpr uint32_t BOOT_DEBOUNCE_MS = 500;
// Earliest moment to cut power after requesting shutdown sound.
constexpr uint32_t POWER_CUT_EARLIEST_MS = 100;
// Hard deadline to cut power even if audio task did not report idle yet.
constexpr uint32_t POWER_CUT_DEADLINE_MS = 320;
// Require SYS_OUT high and stable before accepting any new power long-press.
constexpr uint32_t POWER_IDLE_STABLE_MS = 1200;

constexpr const char *NVS_NAMESPACE = "rf_cfg";
constexpr const char *NVS_KEY_RSSI = "rssi_th";

int rssi_threshold = -60;
bool screen_locked = false;
uint32_t ignore_power_events_until_ms = 0;
bool power_events_armed = false;
uint32_t power_off_allowed_after_ms = 0;
bool power_btn_pressed = false;
uint32_t power_btn_pressed_since_ms = 0;
bool power_seen_released_since_arm = false;
bool power_idle_stable = false;
uint32_t power_idle_since_ms = 0;
bool power_cut_pending = false;
uint32_t power_cut_earliest_ms = 0;
uint32_t power_cut_deadline_ms = 0;
bool boot_btn_pressed = false;

// BOOT button toggles only the backlight (TuneBar behavior).
void set_screen_locked(bool locked) {
    if (locked == screen_locked) {
        return;
    }
    screen_locked = locked;
    if (screen_locked) {
        setUpduty(LCD_PWM_MODE_0);
        Serial.println("[SCREEN] LOCK");
    } else {
        setUpduty(LCD_PWM_MODE_200);
        Serial.println("[SCREEN] UNLOCK");
    }
}

bool usb_connected() {
    return digitalRead(USB_VBUS_GPIO);
}

// Persist RSSI threshold only when value changed.
void save_threshold_to_nvs(int value) {
    static int last_saved = 9999;
    if (value == last_saved) {
        return;
    }

    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, false)) {
        Serial.println("[NVS] Erreur ouverture (write)");
        return;
    }

    prefs.putInt(NVS_KEY_RSSI, value);
    prefs.end();

    last_saved = value;
    Serial.printf("[NVS] Seuil RSSI sauvegarde: %d dBm\n", value);
}

int load_threshold_from_nvs(int default_value) {
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, true)) {
        Serial.println("[NVS] Erreur ouverture (read)");
        return default_value;
    }

    const int value = prefs.getInt(NVS_KEY_RSSI, default_value);
    prefs.end();
    Serial.printf("[NVS] Seuil RSSI charge: %d dBm\n", value);
    return value;
}

void on_threshold_changed(int value) {
    rssi_threshold = value;
}

// Save callback triggered when user leaves threshold settings screen.
void on_threshold_saved(int value) {
    rssi_threshold = value;
    save_threshold_to_nvs(value);
}

void on_splash_done() {
    // Start RF scanning only after splash transition is finished.
    app_state = STATE_SCANNING;
    ESP_LOGI("MAIN", "Splash termine -> SCANNING");
}

void on_battery_update(uint8_t battery_state, float battery_voltage) {
    ui_manager_queue_battery_update(battery_state, battery_voltage);
}

void process_ui_pending_locked() {
    ui_manager_process_pending_update();
}

// Dedicated RF worker:
// - spectrum screen => fast sweep around 433 MHz
// - other screens  => normal detect scan
void rf_task(void *pv) {
    (void)pv;
    bool was_spectrum_mode = false;
    bool prev_signal_detected = false;
    uint32_t last_detect_beep_ms = 0;
    while (true) {
        if (app_state == STATE_SCANNING) {
            if (!ui_manager_is_subghz_active()) {
                if (was_spectrum_mode) {
                    cc1101_manager_restore_scan_mode();
                    was_spectrum_mode = false;
                }
                prev_signal_detected = false;
                vTaskDelay(pdMS_TO_TICKS(SCAN_DELAY_MS));
                continue;
            }

            const bool spectrum_mode = ui_manager_is_spectrum_active();

            // Leaving spectrum can leave radio in a temporary profile, request scan restore.
            if (was_spectrum_mode && !spectrum_mode) {
                cc1101_manager_restore_scan_mode();
            }
            was_spectrum_mode = spectrum_mode;

            if (spectrum_mode) {
                prev_signal_detected = false;
                Cc1101SweepResult sweep{};
                // Sweep feed for spectrum bars.
                if (cc1101_manager_capture_sweep(SWEEP_START_MHZ,
                                                 SWEEP_END_MHZ,
                                                 SWEEP_SAMPLE_COUNT,
                                                 &sweep)) {
                    ui_manager_queue_spectrum_update(sweep);
                }
            } else {
                // Main detection flow used by freq-only and main screens.
                const Cc1101ScanResult result = cc1101_manager_scan_once(rssi_threshold);

                if (result.signal_detected) {
                    const char *mod = result.is_fsk ? "FSK" : "ASK/OOK";
                    ui_manager_set_last_signal(result.detected_freq_mhz, result.detected_rssi_dbm, mod);
                    ui_manager_queue_update(result.detected_freq_mhz,
                                            result.detected_rssi_dbm,
                                            mod,
                                            "Signal detecte");

                    const uint32_t now_ms = millis();
                    if (ui_manager_is_freq_only_active() &&
                        (!prev_signal_detected || (now_ms - last_detect_beep_ms) >= DETECT_BEEP_MIN_INTERVAL_MS)) {
                        audio_feedback_play_detect();
                        last_detect_beep_ms = now_ms;
                    }
                    prev_signal_detected = true;
                } else if ((result.scan_count % 10) == 0) {
                    // Keep UI alive with periodic status while no signal is found.
                    char status_buf[64];
                    snprintf(status_buf, sizeof(status_buf), "Scan #%d - En attente...", result.scan_count);
                    ui_manager_queue_update(0.0f, result.best_rssi_dbm, "---", status_buf);
                    prev_signal_detected = false;
                } else {
                    prev_signal_detected = false;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(SCAN_DELAY_MS));
    }
}

void print_banner() {
    Serial.println("\n\n");
    Serial.println("╔═══════════════════════════════════════════════════╗");
    Serial.println("║                                                   ║");
    Serial.println("║       🔊  DETECTEUR RF SUB-GHz  🔊               ║");
    Serial.println("║                                                   ║");
    Serial.println("║  Scan automatique des frequences 300-928 MHz     ║");
    Serial.println("║  Detection ASK/OOK et FSK                        ║");
    Serial.println("║                                                   ║");
    Serial.println("╚═══════════════════════════════════════════════════╝");
    Serial.println();
}

void enter_deep_sleep_now() {
    // Drop latch only right before deep sleep to guarantee controlled power-off.
    power_manager_commit_power_off();
    esp_sleep_enable_ext1_wakeup(1ULL << POWER_BUTTON_GPIO, ESP_EXT1_WAKEUP_ALL_LOW);
    esp_deep_sleep_start();
}

}  // namespace

void setup() {
    // Hold power latch as early as possible to survive battery-only reset transitions.
    Serial.begin(115200);
    delay(50);
    i2c_master_Init();
    power_manager_init();
    power_manager_on();
    pinMode(USB_VBUS_GPIO, INPUT);
    pinMode(SYS_OUT_GPIO, INPUT_PULLUP);
    pinMode(BOOT_BUTTON_GPIO, INPUT_PULLUP);
    // Ignore early false long-press events right after boot.
    const esp_reset_reason_t rr = esp_reset_reason();
    const bool ext_reset = (rr == ESP_RST_EXT);
    ignore_power_events_until_ms = millis() + (ext_reset ? POWER_EVENTS_ARM_DELAY_EXT_RESET_MS : POWER_EVENTS_ARM_DELAY_MS);
    if (ext_reset) {
        Serial.println("[POWER] External reset detected, extended arm delay");
    }
    if (ext_reset && !usb_connected()) {
        power_off_allowed_after_ms = millis() + POWER_OFF_GUARD_EXT_RESET_BATTERY_MS;
        Serial.println("[POWER] Battery reset guard enabled");
    } else {
        power_off_allowed_after_ms = 0;
    }

    lcd_bl_pwm_bsp_init(LCD_PWM_MODE_255);
    setUpduty(LCD_PWM_MODE_200);
    screen_locked = false;

    print_banner();

    rssi_threshold = load_threshold_from_nvs(-60);

    // Radio must be ready before UI starts consuming scan data.
    if (!cc1101_manager_init(rssi_threshold)) {
        Serial.println("\nERREUR FATALE: Impossible d'initialiser le CC1101");
        while (1) {
            delay(1000);
        }
    }

    audio_feedback_init();
    audio_feedback_play_startup();
    uint32_t audio_wait_start = millis();
    while (!audio_feedback_is_idle() && (millis() - audio_wait_start < STARTUP_SOUND_WAIT_MS)) {
        vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
    }

    Serial.println("\nSysteme pret");
    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    Serial.println("Debut du scan...\n");

    lvgl_port_init();
    // Create UI while holding LVGL internal mutex.
    lvgl_port_run_with_gui([]() {
        ui_manager_init(rssi_threshold, on_threshold_changed, on_threshold_saved);
        ui_manager_create_splash(on_splash_done);
    });
    battery_manager_init(on_battery_update);

    xTaskCreatePinnedToCore(
        rf_task,
        "rf_task",
        12288,
        nullptr,
        3,
        nullptr,
        1
    );
    app_state = STATE_SPLASH;
}

void loop() {
    // Apply queued UI updates from RF task under LVGL mutex.
    lvgl_port_run_with_gui(process_ui_pending_locked);

    // Arm power button events only after startup is fully stable.
    if (!power_events_armed &&
        app_state == STATE_SCANNING &&
        millis() >= ignore_power_events_until_ms) {
        power_events_armed = true;
        power_seen_released_since_arm = false;
        power_idle_stable = false;
        power_idle_since_ms = millis();
        Serial.println("[POWER] Events armed");
    }

    // TuneBar power behavior: long-press SYS_OUT then cut latch directly.
    if (power_events_armed) {
        const bool pressed_now = (digitalRead(SYS_OUT_GPIO) == LOW);
        const uint32_t now_ms = millis();

        if (!pressed_now) {
            power_seen_released_since_arm = true;
            if (!power_idle_stable) {
                if ((now_ms - power_idle_since_ms) >= POWER_IDLE_STABLE_MS) {
                    power_idle_stable = true;
                }
            }
        } else {
            power_idle_since_ms = now_ms;
        }

        if (pressed_now && power_seen_released_since_arm && power_idle_stable && !power_cut_pending) {
            if (!power_btn_pressed) {
                power_btn_pressed = true;
                power_btn_pressed_since_ms = now_ms;
            } else if (now_ms - power_btn_pressed_since_ms >= POWER_HOLD_MS) {
                if (usb_connected()) {
                    Serial.println("[POWER] USB present");
                } else if (power_off_allowed_after_ms && (now_ms < power_off_allowed_after_ms)) {
                    Serial.println("[POWER] OFF blocked by reset guard");
                } else {
                    // Shutdown is two-step: request off, then wait a short window for audio.
                    Serial.println("[POWER] OFF");
                    power_manager_off_request();
                    audio_feedback_play_shutdown();
                    power_cut_pending = true;
                    power_cut_earliest_ms = now_ms + POWER_CUT_EARLIEST_MS;
                    power_cut_deadline_ms = now_ms + POWER_CUT_DEADLINE_MS;
                }
                power_btn_pressed_since_ms = now_ms;
            }
        } else if (power_btn_pressed) {
            power_btn_pressed = false;
        }

        if (power_cut_pending) {
            const bool cut_time_reached = (now_ms >= power_cut_deadline_ms);
            const bool audio_done = audio_feedback_is_idle() && (now_ms >= power_cut_earliest_ms);
            if (cut_time_reached || audio_done) {
                power_manager_commit_power_off();
                while (true) {
                    vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
                }
            }
        }

        // TuneBar BOOT behavior: toggle backlight/screen lock with debounce.
        const bool boot_pressed_now = (digitalRead(BOOT_BUTTON_GPIO) == LOW);
        if (boot_pressed_now && !boot_btn_pressed) {
            boot_btn_pressed = true;
            vTaskDelay(pdMS_TO_TICKS(BOOT_DEBOUNCE_MS));
            set_screen_locked(!screen_locked);
        } else if (!boot_pressed_now && boot_btn_pressed) {
            boot_btn_pressed = false;
        }
    }

    vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
}
