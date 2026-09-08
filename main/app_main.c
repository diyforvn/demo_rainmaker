#include <stdlib.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <nvs_flash.h>

#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_params.h>
#include <esp_rmaker_standard_devices.h>
#include <esp_rmaker_standard_types.h>
#include <esp_rmaker_schedule.h>
#include <esp_rmaker_scenes.h>

#include <app_network.h>
#include <app_insights.h>

#include "app_priv.h"

static const char *TAG = "app_main";

static esp_rmaker_device_t *output_devices[PLC_OUTPUT_COUNT];
static esp_rmaker_device_t *input_devices[PLC_INPUT_COUNT];
static esp_rmaker_param_t *input_state_params[PLC_INPUT_COUNT];

static esp_rmaker_device_t *modbus_cfg_devices[PLC_RS485_COUNT];
static esp_rmaker_param_t *modbus_slave_addr_params[PLC_RS485_COUNT];
static esp_rmaker_param_t *modbus_baud_params[PLC_RS485_COUNT];
static esp_rmaker_param_t *modbus_data_bits_params[PLC_RS485_COUNT];
static esp_rmaker_param_t *modbus_parity_params[PLC_RS485_COUNT];
static esp_rmaker_param_t *modbus_stop_bits_params[PLC_RS485_COUNT];

#define INPUT_DEVICE_TYPE      "esp.device.contact-sensor"
#define INPUT_STATE_PARAM_NAME "Contact Detection State"
#define INPUT_STATE_PARAM_TYPE "esp.param.contact-detection-state"

static int get_int_param_value(const esp_rmaker_param_t *param, int default_value)
{
    if (!param) {
        return default_value;
    }
    esp_rmaker_param_val_t *val = esp_rmaker_param_get_val((esp_rmaker_param_t *)param);
    if (!val) {
        return default_value;
    }
    return val->val.i;
}

static int get_pending_int_param_value(const esp_rmaker_param_t *param, const esp_rmaker_param_t *pending_param,
                                       esp_rmaker_param_val_t pending_val, int default_value)
{
    if (param == pending_param) {
        return pending_val.val.i;
    }
    return get_int_param_value(param, default_value);
}

static esp_err_t modbus_config_write_cb(const esp_rmaker_device_t *device, const esp_rmaker_param_t *param,
                                       const esp_rmaker_param_val_t val, void *priv_data,
                                       esp_rmaker_write_ctx_t *ctx)
{
    (void)device;
    (void)ctx;

    int port = (int)(intptr_t)priv_data;
    if (port < 0 || port >= PLC_RS485_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    int slave_addr = get_pending_int_param_value(modbus_slave_addr_params[port], param, val, 1);
    int baud_rate = get_pending_int_param_value(modbus_baud_params[port], param, val, 9600);
    int data_bits = get_pending_int_param_value(modbus_data_bits_params[port], param, val, 8);
    int parity = get_pending_int_param_value(modbus_parity_params[port], param, val, 0);
    int stop_bits = get_pending_int_param_value(modbus_stop_bits_params[port], param, val, 1);

    app_driver_update_modbus_config(port, slave_addr, baud_rate, data_bits, parity, stop_bits);
    esp_rmaker_param_update_and_report(param, val);
    return ESP_OK;
}

static void create_modbus_config_device(esp_rmaker_node_t *node, int port)
{
    char dev_name[32];
    snprintf(dev_name, sizeof(dev_name), "RS485 %d Config", port + 1);

    modbus_cfg_devices[port] = esp_rmaker_device_create(dev_name, "esp.device.modbus", (void *)(intptr_t)port);

    modbus_slave_addr_params[port] = esp_rmaker_param_create("Slave Address",
                                                            "esp.param.slaveaddress",
                                                            esp_rmaker_int(1),
                                                            PROP_FLAG_READ | PROP_FLAG_WRITE | PROP_FLAG_PERSIST);
    modbus_baud_params[port] = esp_rmaker_param_create("Baudrate",
                                                      "esp.param.baudrate",
                                                      esp_rmaker_int(9600),
                                                      PROP_FLAG_READ | PROP_FLAG_WRITE | PROP_FLAG_PERSIST);
    modbus_data_bits_params[port] = esp_rmaker_param_create("Data Bits",
                                                           "esp.param.databits",
                                                           esp_rmaker_int(8),
                                                           PROP_FLAG_READ | PROP_FLAG_WRITE | PROP_FLAG_PERSIST);
    modbus_parity_params[port] = esp_rmaker_param_create("Parity",
                                                         "esp.param.parity",
                                                         esp_rmaker_int(0),
                                                         PROP_FLAG_READ | PROP_FLAG_WRITE | PROP_FLAG_PERSIST);
    modbus_stop_bits_params[port] = esp_rmaker_param_create("Stop Bits",
                                                            "esp.param.stopbits",
                                                            esp_rmaker_int(1),
                                                            PROP_FLAG_READ | PROP_FLAG_WRITE | PROP_FLAG_PERSIST);

    ESP_ERROR_CHECK(esp_rmaker_device_add_param(modbus_cfg_devices[port], modbus_slave_addr_params[port]));
    ESP_ERROR_CHECK(esp_rmaker_device_add_param(modbus_cfg_devices[port], modbus_baud_params[port]));
    ESP_ERROR_CHECK(esp_rmaker_device_add_param(modbus_cfg_devices[port], modbus_data_bits_params[port]));
    ESP_ERROR_CHECK(esp_rmaker_device_add_param(modbus_cfg_devices[port], modbus_parity_params[port]));
    ESP_ERROR_CHECK(esp_rmaker_device_add_param(modbus_cfg_devices[port], modbus_stop_bits_params[port]));
    ESP_ERROR_CHECK(esp_rmaker_device_add_cb(modbus_cfg_devices[port], modbus_config_write_cb, NULL));
    ESP_ERROR_CHECK(esp_rmaker_node_add_device(node, modbus_cfg_devices[port]));
}

static void create_modbus_config_devices(esp_rmaker_node_t *node)
{
    for (int port = 0; port < PLC_RS485_COUNT; port++) {
        create_modbus_config_device(node, port);
    }
}

static void sync_modbus_config_from_params(void)
{
    for (int port = 0; port < PLC_RS485_COUNT; port++) {
        if (!modbus_cfg_devices[port]) {
            continue;
        }

        int slave_addr = get_int_param_value(modbus_slave_addr_params[port], 1);
        int baud_rate = get_int_param_value(modbus_baud_params[port], 9600);
        int data_bits = get_int_param_value(modbus_data_bits_params[port], 8);
        int parity = get_int_param_value(modbus_parity_params[port], 0);
        int stop_bits = get_int_param_value(modbus_stop_bits_params[port], 1);

        app_driver_update_modbus_config(port, slave_addr, baud_rate, data_bits, parity, stop_bits);
    }
}

void app_driver_notify_output_change(int index, bool state)
{
    if (index < 0 || index >= PLC_OUTPUT_COUNT || !output_devices[index]) {
        return;
    }
    esp_rmaker_param_t *power_param = esp_rmaker_device_get_param_by_name(output_devices[index], ESP_RMAKER_DEF_POWER_NAME);
    if (power_param) {
        esp_rmaker_param_update_and_report(power_param, esp_rmaker_bool(state));
    }
}

void app_driver_notify_input_change(int index, bool state)
{
    if (index < 0 || index >= PLC_INPUT_COUNT || !input_state_params[index]) {
        return;
    }
    esp_rmaker_param_update_and_report(input_state_params[index], esp_rmaker_bool(state));
    app_driver_set_status(APP_LED_STATUS_INPUT_ACTIVITY);
}

static esp_err_t output_write_cb(const esp_rmaker_device_t *device, const esp_rmaker_param_t *param,
                                 const esp_rmaker_param_val_t val, void *priv_data,
                                 esp_rmaker_write_ctx_t *ctx)
{
    (void)device;
    (void)ctx;

    int index = (int)(intptr_t)priv_data;

    ESP_LOGI(TAG, "Set output %d -> %s", index + 1, val.val.b ? "ON" : "OFF");
    app_driver_set_output_state(index, val.val.b);
    app_driver_set_status(APP_LED_STATUS_OUTPUT_ACTIVITY);
    esp_rmaker_param_update_and_report(param, val);

    return ESP_OK;
}

static void create_output_devices(esp_rmaker_node_t *node)
{
    for (int i = 0; i < PLC_OUTPUT_COUNT; i++) {
        char dev_name[16];
        snprintf(dev_name, sizeof(dev_name), "Output %d", i + 1);

        output_devices[i] = esp_rmaker_switch_device_create(dev_name, (void *)(intptr_t)i, false);
        ESP_ERROR_CHECK(esp_rmaker_device_add_cb(output_devices[i], output_write_cb, NULL));
        ESP_ERROR_CHECK(esp_rmaker_node_add_device(node, output_devices[i]));
    }
}

static void create_input_devices(esp_rmaker_node_t *node)
{
    for (int i = 0; i < PLC_INPUT_COUNT; i++) {
        char dev_name[16];
        snprintf(dev_name, sizeof(dev_name), "Input %d", i + 1);

        input_devices[i] = esp_rmaker_device_create(dev_name, INPUT_DEVICE_TYPE, (void *)(intptr_t)i);
        ESP_ERROR_CHECK(esp_rmaker_device_add_param(input_devices[i],
                                                    esp_rmaker_name_param_create(ESP_RMAKER_DEF_NAME_PARAM,
                                                                                 dev_name)));
        input_state_params[i] = esp_rmaker_param_create(INPUT_STATE_PARAM_NAME,
                                                        INPUT_STATE_PARAM_TYPE,
                                                        esp_rmaker_bool(app_driver_get_input_state(i)),
                                                        PROP_FLAG_READ);
        ESP_ERROR_CHECK(esp_rmaker_param_add_ui_type(input_state_params[i], ESP_RMAKER_UI_TOGGLE));
        ESP_ERROR_CHECK(esp_rmaker_device_add_param(input_devices[i], input_state_params[i]));
        ESP_ERROR_CHECK(esp_rmaker_device_assign_primary_param(input_devices[i], input_state_params[i]));
        ESP_ERROR_CHECK(esp_rmaker_node_add_device(node, input_devices[i]));
    }
}

void app_main(void)
{
    esp_err_t err;

    app_driver_init();

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    app_network_init();

    esp_rmaker_config_t rainmaker_cfg = {
        .enable_time_sync = false,
    };

    esp_rmaker_node_t *node = esp_rmaker_node_init(&rainmaker_cfg, "ESP PLC", "esp.plc");
    if (!node) {
        ESP_LOGE(TAG, "Could not initialise node");
        abort();
    }

    create_output_devices(node);
    create_input_devices(node);
    create_modbus_config_devices(node);

    esp_rmaker_ota_enable_default();
    esp_rmaker_timezone_service_enable();
    esp_rmaker_schedule_enable();
    esp_rmaker_scenes_enable();
    app_insights_enable();

    ESP_ERROR_CHECK(esp_rmaker_start());
    sync_modbus_config_from_params();

    err = app_network_start(POP_TYPE_NONE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not start Wi-Fi");
        abort();
    }

    app_driver_set_status(APP_LED_STATUS_BOOT);
}
