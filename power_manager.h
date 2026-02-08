#pragma once

void power_manager_init();

void power_manager_on();
void power_manager_off_request();

bool power_manager_is_device_on();

bool power_manager_handle_button2_release(bool usb_connected);
void power_manager_commit_power_off();

