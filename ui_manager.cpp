#include "ui_manager.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <stdio.h>
#include <string.h>

LV_FONT_DECLARE(ui_font_zendots115);
LV_FONT_DECLARE(ui_font_zendots59);

namespace {

// Internal screen ids used by swipe navigation.
enum UiScreenInternal {
    SCREEN_SPLASH = 0,
    SCREEN_MENU,
    SCREEN_FREQ_ONLY,
    SCREEN_MAIN,
    SCREEN_SPECTRUM,
    SCREEN_IR,
    SCREEN_THRESHOLD,
};

constexpr int kScreenWidth = 640;
constexpr int kScreenHeight = 172;
constexpr uint16_t kSpectrumPointCount = 96;
constexpr int kSpectrumPlotW = 620;
constexpr int kSpectrumPlotH = 98;
constexpr int kSpectrumRssiMin = -110;
constexpr int kSpectrumRssiMax = -35;

lv_obj_t *main_screen = nullptr;
lv_obj_t *freq_label = nullptr;
lv_obj_t *rssi_label = nullptr;
lv_obj_t *mod_label = nullptr;
lv_obj_t *status_label = nullptr;
lv_obj_t *threshold_label = nullptr;
lv_obj_t *history_label = nullptr;
lv_obj_t *battery_label = nullptr;

lv_obj_t *screen_freq_only = nullptr;
lv_obj_t *freq_only_label = nullptr;

lv_obj_t *screen_menu = nullptr;
lv_obj_t *menu_title_label = nullptr;
lv_obj_t *menu_card_subghz = nullptr;
lv_obj_t *menu_card_ir = nullptr;
lv_obj_t *menu_hint_label = nullptr;

lv_obj_t *screen_spectrum = nullptr;
lv_obj_t *spectrum_info_label = nullptr;
lv_obj_t *spectrum_plot = nullptr;
lv_obj_t *spectrum_bars[kSpectrumPointCount] = {nullptr};

lv_obj_t *screen_ir = nullptr;

lv_obj_t *screen_threshold = nullptr;
lv_obj_t *threshold_slider = nullptr;
lv_obj_t *threshold_value_label = nullptr;

lv_obj_t *splash_screen = nullptr;
lv_timer_t *splash_timer = nullptr;

portMUX_TYPE ui_data_mux = portMUX_INITIALIZER_UNLOCKED;
// Pending data exchanged between RF task and UI thread.
volatile bool ui_needs_update = false;
float pending_freq = 0.0f;
int pending_rssi = 0;
char pending_mod[32] = "";
char pending_status[64] = "";

volatile bool spectrum_needs_update = false;
Cc1101SweepResult pending_spectrum = {};

volatile bool battery_needs_update = false;
uint8_t pending_battery_state = 0;
float pending_battery_voltage = 0.0f;

float last_freq_mhz = 0.0f;
int last_rssi_dbm = -120;
char last_modulation[16] = "----";

int rssi_threshold = -60;
UiThresholdChangedCb on_threshold_changed = nullptr;
UiThresholdSavedCb on_threshold_saved = nullptr;
UiSplashDoneCb on_splash_done = nullptr;
volatile UiScreenInternal active_screen = SCREEN_SPLASH;

void load_screen(lv_obj_t *screen, UiScreenInternal screen_id) {
    if (!screen) {
        return;
    }
    lv_scr_load(screen);
    active_screen = screen_id;
}

void update_ui(float freq_mhz, int rssi, const char *modulation, const char *status) {
    char buf[96];

    if (freq_only_label) {
        snprintf(buf, sizeof(buf), (freq_mhz > 0.01f) ? "%.2f" : "----", freq_mhz);
        lv_label_set_text(freq_only_label, buf);
    }

    if (freq_label) {
        snprintf(buf, sizeof(buf), (freq_mhz > 0.01f) ? "%.2f" : "----", freq_mhz);
        lv_label_set_text(freq_label, buf);
    }

    if (rssi_label) {
        snprintf(buf, sizeof(buf), "RSSI: %d dBm", rssi);
        lv_label_set_text(rssi_label, buf);
    }

    if (mod_label && modulation) {
        lv_label_set_text(mod_label, modulation);
    }

    if (status_label && status) {
        lv_label_set_text(status_label, status);
    }

    if (history_label) {
        if (last_freq_mhz > 0.01f) {
            snprintf(buf, sizeof(buf), "Dernier: %.2f MHz | %d dBm | %s",
                     last_freq_mhz, last_rssi_dbm, last_modulation);
        } else {
            snprintf(buf, sizeof(buf), "Dernier: aucun signal");
        }
        lv_label_set_text(history_label, buf);
    }
}

const char *battery_symbol_for_state(uint8_t state) {
    switch (state) {
        case 4:
            return LV_SYMBOL_BATTERY_FULL;
        case 3:
            return LV_SYMBOL_BATTERY_3;
        case 2:
            return LV_SYMBOL_BATTERY_2;
        case 1:
            return LV_SYMBOL_BATTERY_1;
        default:
            return LV_SYMBOL_BATTERY_EMPTY;
    }
}

void update_battery_ui(uint8_t battery_state, float battery_voltage) {
    (void)battery_voltage;
    if (!battery_label) {
        return;
    }
    lv_label_set_text(battery_label, battery_symbol_for_state(battery_state));
}

int clamp_int(int value, int min_value, int max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

int rssi_to_bar_height(int rssi_dbm) {
    const int clamped = clamp_int(rssi_dbm, kSpectrumRssiMin, kSpectrumRssiMax);
    const int range = kSpectrumRssiMax - kSpectrumRssiMin;
    if (range <= 0) {
        return 2;
    }
    const int mapped = (clamped - kSpectrumRssiMin) * (kSpectrumPlotH - 2) / range;
    return clamp_int(mapped, 2, kSpectrumPlotH);
}

void update_spectrum_visual(const Cc1101SweepResult &sweep) {
    if (!sweep.valid || sweep.sample_count < 2 || !spectrum_plot) {
        return;
    }

    const bool signal_detected = (sweep.max_rssi_dbm >= rssi_threshold);
    const lv_color_t color = signal_detected ? lv_color_hex(0xF05A28) : lv_color_hex(0x1E88E5);

    // Resample RF sweep to fixed UI bin count.
    for (uint16_t i = 0; i < kSpectrumPointCount; ++i) {
        const uint16_t src_idx = static_cast<uint16_t>(
            (static_cast<uint32_t>(i) * (sweep.sample_count - 1)) / (kSpectrumPointCount - 1));
        const int h = rssi_to_bar_height(sweep.rssi_dbm[src_idx]);
        if (spectrum_bars[i]) {
            // Bars are anchored at the bottom of the plot area.
            lv_obj_set_height(spectrum_bars[i], h);
            lv_obj_set_y(spectrum_bars[i], kSpectrumPlotH - h);
            lv_obj_set_style_bg_color(spectrum_bars[i], color, 0);
        }
    }

    if (spectrum_info_label) {
        char info[96];
        snprintf(info,
                 sizeof(info),
                 "%s | Max %.3f MHz | %d dBm",
                 signal_detected ? "Signal detecte" : "En attente",
                 sweep.max_freq_mhz,
                 sweep.max_rssi_dbm);
        lv_label_set_text(spectrum_info_label, info);
    }
}

void menu_subghz_card_event_cb(lv_event_t *e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    load_screen(screen_freq_only, SCREEN_FREQ_ONLY);
}

void menu_ir_card_event_cb(lv_event_t *e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    load_screen(screen_ir, SCREEN_IR);
}

void ir_gesture_event_cb(lv_event_t *e) {
    if (lv_event_get_code(e) != LV_EVENT_GESTURE) {
        return;
    }

    const lv_dir_t dir = lv_indev_get_gesture_dir(lv_indev_get_act());
    if (dir == LV_DIR_BOTTOM) {
        load_screen(screen_menu, SCREEN_MENU);
    }
}

void swipe_event_cb(lv_event_t *e) {
    if (lv_event_get_code(e) != LV_EVENT_GESTURE) {
        return;
    }

    lv_dir_t dir = lv_indev_get_gesture_dir(lv_indev_get_act());
    if (dir == LV_DIR_RIGHT) {
        // Right swipe moves forward in UI flow.
        if (active_screen == SCREEN_FREQ_ONLY) {
            load_screen(main_screen, SCREEN_MAIN);
        } else if (active_screen == SCREEN_MAIN) {
            load_screen(screen_spectrum, SCREEN_SPECTRUM);
        }
    } else if (dir == LV_DIR_LEFT) {
        // Left swipe moves backward in UI flow.
        if (active_screen == SCREEN_SPECTRUM) {
            load_screen(main_screen, SCREEN_MAIN);
        } else if (active_screen == SCREEN_MAIN) {
            load_screen(screen_freq_only, SCREEN_FREQ_ONLY);
        }
    } else if (dir == LV_DIR_TOP) {
        // Up swipe from freq-only opens threshold settings.
        if (active_screen == SCREEN_FREQ_ONLY) {
            load_screen(screen_threshold, SCREEN_THRESHOLD);
        }
    } else if (dir == LV_DIR_BOTTOM) {
        if (active_screen == SCREEN_FREQ_ONLY ||
            active_screen == SCREEN_MAIN ||
            active_screen == SCREEN_SPECTRUM ||
            active_screen == SCREEN_THRESHOLD) {
            load_screen(screen_menu, SCREEN_MENU);
        }
    }
}

void threshold_slider_event_cb(lv_event_t *e) {
    rssi_threshold = lv_slider_get_value((lv_obj_t *)lv_event_get_target(e));

    char buf[24];
    snprintf(buf, sizeof(buf), "%d dBm", rssi_threshold);
    lv_label_set_text(threshold_value_label, buf);

    if (on_threshold_changed) {
        on_threshold_changed(rssi_threshold);
    }
}

void threshold_back_btn_cb(lv_event_t *e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }

    if (on_threshold_saved) {
        on_threshold_saved(rssi_threshold);
    }

    if (threshold_label) {
        char buf[32];
        snprintf(buf, sizeof(buf), "Seuil: %d dBm", rssi_threshold);
        lv_label_set_text(threshold_label, buf);
    }

    load_screen(screen_freq_only, SCREEN_FREQ_ONLY);
}

void splash_timer_cb(lv_timer_t *timer) {
    (void)timer;

    if (screen_menu) {
        load_screen(screen_menu, SCREEN_MENU);
    }

    if (splash_screen) {
        lv_obj_del(splash_screen);
        splash_screen = nullptr;
    }

    if (splash_timer) {
        lv_timer_del(splash_timer);
        splash_timer = nullptr;
    }

    if (on_splash_done) {
        on_splash_done();
    }
}

void create_menu_screen() {
    screen_menu = lv_obj_create(nullptr);
    lv_obj_set_size(screen_menu, kScreenWidth, kScreenHeight);
    lv_obj_set_style_bg_color(screen_menu, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(screen_menu, LV_OPA_COVER, 0);

    menu_title_label = lv_label_create(screen_menu);
    lv_label_set_text(menu_title_label, "Choisir une App");
    lv_obj_set_style_text_font(menu_title_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(menu_title_label, lv_color_black(), 0);
    lv_obj_align(menu_title_label, LV_ALIGN_TOP_MID, 0, 4);

    menu_card_subghz = lv_btn_create(screen_menu);
    lv_obj_set_size(menu_card_subghz, 300, 104);
    lv_obj_align(menu_card_subghz, LV_ALIGN_CENTER, -156, 10);
    lv_obj_set_style_radius(menu_card_subghz, 14, 0);
    lv_obj_set_style_bg_color(menu_card_subghz, lv_color_hex(0x1E88E5), 0);
    lv_obj_add_event_cb(menu_card_subghz, menu_subghz_card_event_cb, LV_EVENT_CLICKED, nullptr);

    lv_obj_t *subghz_title = lv_label_create(menu_card_subghz);
    lv_label_set_text(subghz_title, "SubGHz");
    lv_obj_set_style_text_font(subghz_title, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(subghz_title, lv_color_white(), 0);
    lv_obj_align(subghz_title, LV_ALIGN_TOP_MID, 0, 18);

    lv_obj_t *subghz_subtitle = lv_label_create(menu_card_subghz);
    lv_label_set_text(subghz_subtitle, "RF Detect / Spectrum");
    lv_obj_set_style_text_font(subghz_subtitle, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(subghz_subtitle, lv_color_white(), 0);
    lv_obj_align(subghz_subtitle, LV_ALIGN_BOTTOM_MID, 0, -16);

    menu_card_ir = lv_btn_create(screen_menu);
    lv_obj_set_size(menu_card_ir, 300, 104);
    lv_obj_align(menu_card_ir, LV_ALIGN_CENTER, 156, 10);
    lv_obj_set_style_radius(menu_card_ir, 14, 0);
    lv_obj_set_style_bg_color(menu_card_ir, lv_color_hex(0xF05A28), 0);
    lv_obj_add_event_cb(menu_card_ir, menu_ir_card_event_cb, LV_EVENT_CLICKED, nullptr);

    lv_obj_t *ir_title = lv_label_create(menu_card_ir);
    lv_label_set_text(ir_title, "IR");
    lv_obj_set_style_text_font(ir_title, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(ir_title, lv_color_white(), 0);
    lv_obj_align(ir_title, LV_ALIGN_TOP_MID, 0, 18);

    lv_obj_t *ir_subtitle = lv_label_create(menu_card_ir);
    lv_label_set_text(ir_subtitle, "Infrared");
    lv_obj_set_style_text_font(ir_subtitle, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(ir_subtitle, lv_color_white(), 0);
    lv_obj_align(ir_subtitle, LV_ALIGN_BOTTOM_MID, 0, -16);

    menu_hint_label = lv_label_create(screen_menu);
    lv_label_set_text(menu_hint_label, "Tap pour ouvrir");
    lv_obj_set_style_text_font(menu_hint_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(menu_hint_label, lv_color_hex(0x555555), 0);
    lv_obj_align(menu_hint_label, LV_ALIGN_BOTTOM_MID, 0, -8);
}

void create_ir_screen() {
    screen_ir = lv_obj_create(nullptr);
    lv_obj_set_size(screen_ir, kScreenWidth, kScreenHeight);
    lv_obj_set_style_bg_color(screen_ir, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(screen_ir, LV_OPA_COVER, 0);

    lv_obj_t *title = lv_label_create(screen_ir);
    lv_label_set_text(title, "IR");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(title, lv_color_black(), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 18);

    lv_obj_t *subtitle = lv_label_create(screen_ir);
    lv_label_set_text(subtitle, "Ecran IR pret pour integration");
    lv_obj_set_style_text_font(subtitle, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(subtitle, lv_color_hex(0x333333), 0);
    lv_obj_align(subtitle, LV_ALIGN_CENTER, 0, -4);

    lv_obj_t *hint = lv_label_create(screen_ir);
    lv_label_set_text(hint, "Swipe haut -> bas: retour menu");
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(hint, lv_color_hex(0x666666), 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -8);

    lv_obj_add_event_cb(screen_ir, ir_gesture_event_cb, LV_EVENT_GESTURE, nullptr);
}

void create_threshold_screen() {
    screen_threshold = lv_obj_create(nullptr);
    lv_obj_set_style_bg_color(screen_threshold, lv_color_white(), 0);

    lv_obj_t *back_btn = lv_btn_create(screen_threshold);
    lv_obj_set_size(back_btn, 60, 40);
    lv_obj_align(back_btn, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_add_event_cb(back_btn, threshold_back_btn_cb, LV_EVENT_CLICKED, nullptr);

    lv_obj_t *back_lbl = lv_label_create(back_btn);
    lv_label_set_text(back_lbl, LV_SYMBOL_LEFT);
    lv_obj_center(back_lbl);

    threshold_value_label = lv_label_create(screen_threshold);
    char buf[24];
    snprintf(buf, sizeof(buf), "%d dBm", rssi_threshold);
    lv_label_set_text(threshold_value_label, buf);
    lv_obj_align(threshold_value_label, LV_ALIGN_TOP_MID, 0, 20);

    threshold_slider = lv_slider_create(screen_threshold);
    lv_slider_set_range(threshold_slider, -120, -30);
    lv_slider_set_value(threshold_slider, rssi_threshold, LV_ANIM_OFF);
    lv_obj_set_size(threshold_slider, 580, 30);
    lv_obj_align(threshold_slider, LV_ALIGN_CENTER, 0, 10);
    lv_obj_add_event_cb(threshold_slider, threshold_slider_event_cb, LV_EVENT_VALUE_CHANGED, nullptr);

    lv_obj_add_event_cb(screen_threshold, swipe_event_cb, LV_EVENT_GESTURE, nullptr);
}

void create_freq_only_screen() {
    screen_freq_only = lv_obj_create(nullptr);
    lv_obj_set_style_bg_color(screen_freq_only, lv_color_white(), 0);

    freq_only_label = lv_label_create(screen_freq_only);
    lv_label_set_text(freq_only_label, "----");
    lv_obj_set_style_text_font(freq_only_label, &ui_font_zendots115, 0);
    lv_obj_set_style_text_color(freq_only_label, lv_color_black(), 0);
    lv_obj_center(freq_only_label);

    lv_obj_add_event_cb(screen_freq_only, swipe_event_cb, LV_EVENT_GESTURE, nullptr);
}

void create_spectrum_screen() {
    screen_spectrum = lv_obj_create(nullptr);
    lv_obj_set_size(screen_spectrum, kScreenWidth, kScreenHeight);
    lv_obj_set_style_bg_color(screen_spectrum, lv_color_white(), 0);

    lv_obj_t *title = lv_label_create(screen_spectrum);
    lv_label_set_text(title, "RF Spectrum (433 MHz)");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(title, lv_color_black(), 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 8, 4);

    spectrum_info_label = lv_label_create(screen_spectrum);
    lv_label_set_text(spectrum_info_label, "En attente | Max -- MHz | --- dBm");
    lv_obj_set_style_text_font(spectrum_info_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(spectrum_info_label, lv_color_hex(0x333333), 0);
    lv_obj_align(spectrum_info_label, LV_ALIGN_TOP_LEFT, 8, 30);

    lv_obj_t *range_label = lv_label_create(screen_spectrum);
    lv_label_set_text(range_label, "Bande: 433.05 MHz <-> 434.79 MHz");
    lv_obj_set_style_text_font(range_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(range_label, lv_color_hex(0x666666), 0);
    lv_obj_align(range_label, LV_ALIGN_TOP_LEFT, 8, 52);

    // Plot container where custom bar objects are drawn.
    spectrum_plot = lv_obj_create(screen_spectrum);
    lv_obj_set_size(spectrum_plot, kSpectrumPlotW, kSpectrumPlotH);
    lv_obj_align(spectrum_plot, LV_ALIGN_BOTTOM_MID, 0, -6);
    lv_obj_set_style_bg_color(spectrum_plot, lv_color_hex(0xF3F6FA), 0);
    lv_obj_set_style_bg_opa(spectrum_plot, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(spectrum_plot, lv_color_hex(0xCCCCCC), 0);
    lv_obj_set_style_border_width(spectrum_plot, 1, 0);
    lv_obj_set_style_pad_all(spectrum_plot, 0, 0);
    lv_obj_set_style_radius(spectrum_plot, 0, 0);
    lv_obj_clear_flag(spectrum_plot, LV_OBJ_FLAG_SCROLLABLE);

    const int gap = 1;
    int bar_w = (kSpectrumPlotW - (kSpectrumPointCount + 1) * gap) / kSpectrumPointCount;
    if (bar_w < 1) {
        bar_w = 1;
    }
    const int used_w = kSpectrumPointCount * bar_w + (kSpectrumPointCount - 1) * gap;
    int x = (kSpectrumPlotW - used_w) / 2;
    if (x < 0) {
        x = 0;
    }

    for (uint16_t i = 0; i < kSpectrumPointCount; ++i) {
        spectrum_bars[i] = lv_obj_create(spectrum_plot);
        lv_obj_set_size(spectrum_bars[i], bar_w, 2);
        lv_obj_set_pos(spectrum_bars[i], x, kSpectrumPlotH - 2);
        lv_obj_set_style_bg_color(spectrum_bars[i], lv_color_hex(0x1E88E5), 0);
        lv_obj_set_style_bg_opa(spectrum_bars[i], LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(spectrum_bars[i], 0, 0);
        lv_obj_set_style_radius(spectrum_bars[i], 0, 0);
        lv_obj_clear_flag(spectrum_bars[i], LV_OBJ_FLAG_SCROLLABLE);
        x += bar_w + gap;
    }

    lv_obj_add_event_cb(screen_spectrum, swipe_event_cb, LV_EVENT_GESTURE, nullptr);
}

}  // namespace

void ui_manager_init(int initial_threshold,
                     UiThresholdChangedCb threshold_changed_cb,
                     UiThresholdSavedCb threshold_saved_cb) {
    static bool initialized = false;
    if (initialized) {
        return;
    }

    rssi_threshold = initial_threshold;
    on_threshold_changed = threshold_changed_cb;
    on_threshold_saved = threshold_saved_cb;

    main_screen = lv_obj_create(nullptr);
    lv_obj_set_size(main_screen, kScreenWidth, kScreenHeight);
    lv_obj_set_style_bg_color(main_screen, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(main_screen, LV_OPA_COVER, 0);

    freq_label = lv_label_create(main_screen);
    lv_label_set_text(freq_label, "----");
    lv_obj_set_style_text_font(freq_label, &ui_font_zendots59, 0);
    lv_obj_set_style_text_color(freq_label, lv_color_black(), 0);
    lv_obj_align(freq_label, LV_ALIGN_TOP_MID, 0, 10);

    const int col_x = 10;
    const int y_offset = 80;
    const int y_step = 25;

    rssi_label = lv_label_create(main_screen);
    lv_label_set_text(rssi_label, "RSSI: --- dBm");
    lv_obj_set_style_text_color(rssi_label, lv_color_black(), 0);
#if LV_FONT_MONTSERRAT_18
    lv_obj_set_style_text_font(rssi_label, &lv_font_montserrat_18, 0);
#endif
    lv_obj_align(rssi_label, LV_ALIGN_TOP_LEFT, col_x, y_offset);

    mod_label = lv_label_create(main_screen);
    lv_label_set_text(mod_label, "Modulation: ---");
    lv_obj_set_style_text_color(mod_label, lv_color_black(), 0);
#if LV_FONT_MONTSERRAT_18
    lv_obj_set_style_text_font(mod_label, &lv_font_montserrat_18, 0);
#endif
    lv_obj_align(mod_label, LV_ALIGN_TOP_LEFT, col_x, y_offset + y_step);

    history_label = lv_label_create(main_screen);
    lv_label_set_text(history_label, "Dernier: ---");
    lv_obj_set_style_text_color(history_label, lv_color_hex(0x444444), 0);
    lv_obj_set_style_text_font(history_label, &lv_font_montserrat_18, 0);
    lv_obj_align(history_label, LV_ALIGN_TOP_LEFT, col_x, y_offset + 2 * y_step);

    status_label = lv_label_create(main_screen);
    lv_label_set_text(status_label, "En attente...");
    lv_obj_set_style_text_color(status_label, lv_color_black(), 0);
#if LV_FONT_MONTSERRAT_14
    lv_obj_set_style_text_font(status_label, &lv_font_montserrat_14, 0);
#endif
    lv_obj_align(status_label, LV_ALIGN_TOP_LEFT, col_x, y_offset + 3 * y_step);

    threshold_label = lv_label_create(main_screen);
    char buf[32];
    snprintf(buf, sizeof(buf), "Seuil: %d dBm", rssi_threshold);
    lv_label_set_text(threshold_label, buf);
    lv_obj_set_style_text_color(threshold_label, lv_color_black(), 0);
    lv_obj_set_style_text_font(threshold_label, &lv_font_montserrat_18, 0);
    lv_obj_align(threshold_label, LV_ALIGN_BOTTOM_RIGHT, -5, -5);

    battery_label = lv_label_create(main_screen);
    lv_label_set_text(battery_label, LV_SYMBOL_BATTERY_EMPTY);
    lv_obj_set_style_text_font(battery_label, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(battery_label, lv_color_black(), 0);
    lv_obj_align(battery_label, LV_ALIGN_TOP_RIGHT, -10, 10);

    create_menu_screen();
    create_freq_only_screen();
    create_spectrum_screen();
    create_ir_screen();
    create_threshold_screen();

    lv_obj_add_event_cb(main_screen, swipe_event_cb, LV_EVENT_GESTURE, nullptr);

    load_screen(screen_menu, SCREEN_MENU);
    initialized = true;
}

void ui_manager_create_splash(UiSplashDoneCb splash_done_cb) {
    on_splash_done = splash_done_cb;

    splash_screen = lv_obj_create(nullptr);
    lv_obj_set_style_bg_color(splash_screen, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(splash_screen, LV_OPA_COVER, 0);

    lv_obj_t *label = lv_label_create(splash_screen);
    lv_label_set_text(label, "bixente");
    lv_obj_set_style_text_font(label, &ui_font_zendots115, 0);
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_center(label);

    load_screen(splash_screen, SCREEN_SPLASH);
    splash_timer = lv_timer_create(splash_timer_cb, 1500, nullptr);
}

void ui_manager_queue_update(float freq_mhz, int rssi, const char *modulation, const char *status) {
    // Producer side (RF task): just store latest values.
    portENTER_CRITICAL(&ui_data_mux);
    pending_freq = freq_mhz;
    pending_rssi = rssi;

    if (modulation) {
        strncpy(pending_mod, modulation, sizeof(pending_mod) - 1);
        pending_mod[sizeof(pending_mod) - 1] = '\0';
    }

    if (status) {
        strncpy(pending_status, status, sizeof(pending_status) - 1);
        pending_status[sizeof(pending_status) - 1] = '\0';
    }

    ui_needs_update = true;
    portEXIT_CRITICAL(&ui_data_mux);
}

void ui_manager_set_last_signal(float freq_mhz, int rssi, const char *modulation) {
    last_freq_mhz = freq_mhz;
    last_rssi_dbm = rssi;

    if (modulation) {
        strncpy(last_modulation, modulation, sizeof(last_modulation) - 1);
        last_modulation[sizeof(last_modulation) - 1] = '\0';
    }
}

void ui_manager_queue_spectrum_update(const Cc1101SweepResult &sweep) {
    if (!sweep.valid || sweep.sample_count < 2 || sweep.sample_count > CC1101_SWEEP_MAX_SAMPLES) {
        return;
    }

    portENTER_CRITICAL(&ui_data_mux);
    pending_spectrum = sweep;
    spectrum_needs_update = true;
    portEXIT_CRITICAL(&ui_data_mux);
}

void ui_manager_queue_battery_update(uint8_t battery_state, float battery_voltage) {
    portENTER_CRITICAL(&ui_data_mux);
    pending_battery_state = battery_state;
    pending_battery_voltage = battery_voltage;
    battery_needs_update = true;
    portEXIT_CRITICAL(&ui_data_mux);
}

void ui_manager_process_pending_update() {
    // Consumer side (UI thread): copy pending data then render.
    bool do_ui = false;
    float local_freq = 0.0f;
    int local_rssi = 0;
    char local_mod[sizeof(pending_mod)] = "";
    char local_status[sizeof(pending_status)] = "";

    bool do_spectrum = false;
    Cc1101SweepResult local_sweep = {};

    bool do_battery = false;
    uint8_t local_battery_state = 0;
    float local_battery_voltage = 0.0f;

    portENTER_CRITICAL(&ui_data_mux);
    if (ui_needs_update) {
        local_freq = pending_freq;
        local_rssi = pending_rssi;
        strncpy(local_mod, pending_mod, sizeof(local_mod));
        strncpy(local_status, pending_status, sizeof(local_status));
        ui_needs_update = false;
        do_ui = true;
    }

    if (spectrum_needs_update) {
        local_sweep = pending_spectrum;
        spectrum_needs_update = false;
        do_spectrum = true;
    }

    if (battery_needs_update) {
        local_battery_state = pending_battery_state;
        local_battery_voltage = pending_battery_voltage;
        battery_needs_update = false;
        do_battery = true;
    }
    portEXIT_CRITICAL(&ui_data_mux);

    if (do_ui) {
        update_ui(local_freq, local_rssi, local_mod, local_status);
    }

    if (do_spectrum) {
        update_spectrum_visual(local_sweep);
    }

    if (do_battery) {
        update_battery_ui(local_battery_state, local_battery_voltage);
    }
}

bool ui_manager_is_spectrum_active() {
    return (active_screen == SCREEN_SPECTRUM);
}

bool ui_manager_is_subghz_active() {
    return (active_screen == SCREEN_FREQ_ONLY ||
            active_screen == SCREEN_MAIN ||
            active_screen == SCREEN_SPECTRUM ||
            active_screen == SCREEN_THRESHOLD);
}

bool ui_manager_is_freq_only_active() {
    return (active_screen == SCREEN_FREQ_ONLY);
}

int ui_manager_get_threshold() {
    return rssi_threshold;
}
