#include "power_manager.h"

#include "esp_io_expander_tca9554.h"
#include "i2c_bsp.h"
#include "lcd_bl_pwm_bsp.h"

#include <Arduino.h>

namespace {

esp_io_expander_handle_t power_io_expander = nullptr;
bool latch_active = false;
bool amp_active = false;

bool device_on = false;
bool shutdown_pending = false;
bool ignore_release = false;

void latch_on() {
    if (!power_io_expander || latch_active) {
        return;
    }
    esp_io_expander_set_level(power_io_expander, IO_EXPANDER_PIN_NUM_6, 1);
    latch_active = true;
}

void latch_off() {
    if (!power_io_expander || !latch_active) {
        return;
    }
    esp_io_expander_set_level(power_io_expander, IO_EXPANDER_PIN_NUM_6, 0);
    latch_active = false;
}

void amp_on() {
    if (!power_io_expander || amp_active) {
        return;
    }
    esp_io_expander_set_level(power_io_expander, IO_EXPANDER_PIN_NUM_7, 1);
    amp_active = true;
}

void amp_off() {
    if (!power_io_expander || !amp_active) {
        return;
    }
    esp_io_expander_set_level(power_io_expander, IO_EXPANDER_PIN_NUM_7, 0);
    amp_active = false;
}

void init_latch() {
    i2c_master_bus_handle_t bus;
    esp_err_t ret = i2c_master_get_bus_handle(I2C_NUM_0, &bus);
    if (ret != ESP_OK) {
        Serial.println("[LATCH] Erreur: impossible d'obtenir le bus I2C");
        return;
    }

    ret = esp_io_expander_new_i2c_tca9554(
        bus,
        ESP_IO_EXPANDER_I2C_TCA9554_ADDRESS_000,
        &power_io_expander
    );
    if (ret != ESP_OK || !power_io_expander) {
        Serial.println("[LATCH] Erreur: impossible d'initialiser le TCA9554");
        return;
    }

    ret = esp_io_expander_set_dir(power_io_expander, IO_EXPANDER_PIN_NUM_6, IO_EXPANDER_OUTPUT);
    if (ret != ESP_OK) {
        Serial.println("[LATCH] Erreur: impossible de configurer le pin 6");
        return;
    }
    ret = esp_io_expander_set_dir(power_io_expander, IO_EXPANDER_PIN_NUM_7, IO_EXPANDER_OUTPUT);
    if (ret != ESP_OK) {
        Serial.println("[LATCH] Erreur: impossible de configurer le pin 7 (AMP)");
        return;
    }

    esp_io_expander_set_level(power_io_expander, IO_EXPANDER_PIN_NUM_6, 1);
    esp_io_expander_set_level(power_io_expander, IO_EXPANDER_PIN_NUM_7, 1);
    latch_active = true;
    amp_active = true;
    Serial.println("[LATCH] Latch initialise et active");
    Serial.println("[AMP] ON");
}

void backlight_on() {
    setUpduty(LCD_PWM_MODE_255);
}

void backlight_off() {
    setUpduty(LCD_PWM_MODE_0);
}

}  // namespace

void power_manager_init() {
    init_latch();
}

void power_manager_on() {
    latch_on();
    amp_on();
    backlight_on();
    device_on = true;
    ignore_release = true;
    Serial.println("[POWER] ON");
}

void power_manager_off_request() {
    shutdown_pending = true;
    backlight_off();
    device_on = false;
    Serial.println("[POWER] OFF REQUEST");
}

bool power_manager_is_device_on() {
    return device_on;
}

bool power_manager_handle_button2_release(bool usb_connected) {
    if (ignore_release) {
        ignore_release = false;
        return false;
    }

    if (!shutdown_pending) {
        return false;
    }

    shutdown_pending = false;

    if (usb_connected) {
        Serial.println("[POWER] USB present");
        return false;
    }

    return true;
}

void power_manager_commit_power_off() {
    amp_off();
    latch_off();
}
