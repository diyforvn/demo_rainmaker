#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <esp_err.h>
#include <esp_rmaker_core.h>

#define PLC_INPUT_COUNT   4
#define PLC_OUTPUT_COUNT  4
#define PLC_RS485_COUNT   2

typedef enum {
    APP_LED_STATUS_BOOT = 0,
    APP_LED_STATUS_WIFI_CONNECTING,
    APP_LED_STATUS_ONLINE,
    APP_LED_STATUS_OUTPUT_ACTIVITY,
    APP_LED_STATUS_INPUT_ACTIVITY,
    APP_LED_STATUS_ERROR,
} app_led_status_t;

void app_driver_init(void);

void app_driver_set_output_state(int index, bool state);
void app_driver_notify_output_change(int index, bool state);
void app_driver_notify_input_change(int index, bool state);
void app_driver_update_modbus_config(int port, int slave_addr, int baud_rate, int data_bits, int parity, int stop_bits);
bool app_driver_get_output_state(int index);

bool app_driver_get_input_state(int index);

esp_err_t app_driver_rs485_write(int port, const uint8_t *data, size_t len);

void app_driver_set_status(app_led_status_t status);
void app_driver_pulse_status(app_led_status_t status, uint32_t duration_ms);
