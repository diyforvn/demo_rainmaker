#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/uart.h>
#include <esp_log.h>
#include <esp_err.h>
#include <esp_check.h>
#include <esp_timer.h>

#include <led_strip.h>
#include <led_strip_rmt.h>

#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_types.h>
#include <esp_rmaker_standard_params.h>

#include <iot_button.h>
#include "app_reset.h"

#include "app_priv.h"

#define TAG "app_driver"

#define STATUS_LED_GPIO             2
#define STATUS_LED_COUNT            1
#define STATUS_LED_BRIGHTNESS       16

#define WIFI_RESET_TIMEOUT_S        3
#define FACTORY_RESET_TIMEOUT_S     10
#define RESET_BLINK_INTERVAL_MS     200

#define BOOT_BUTTON_GPIO            0
#define BOOT_BUTTON_ACTIVE_LEVEL    0

#define RS485_PORT_1_UART           UART_NUM_1
#define RS485_PORT_2_UART           UART_NUM_2

#define RS485_PORT_1_TX_GPIO        25
#define RS485_PORT_1_RX_GPIO        26
#define RS485_PORT_2_TX_GPIO        17
#define RS485_PORT_2_RX_GPIO        16

#define RS485_PORT_1_DE_GPIO        27
#define RS485_PORT_2_DE_GPIO        4

#define RS485_BAUD_RATE             9600
#define RS485_DATA_BITS             UART_DATA_8_BITS
#define RS485_PARITY                UART_PARITY_DISABLE
#define RS485_STOP_BITS             UART_STOP_BITS_1
#define RS485_FLOW_CTRL             UART_HW_FLOWCTRL_DISABLE

#define MODBUS_SLAVE_ADDR           1
#define MODBUS_RX_TIMEOUT_MS        100
#define MODBUS_TASK_STACK_SIZE      4096
#define MODBUS_TASK_PRIORITY        5

#define INPUT_POLL_INTERVAL_MS      50
#define INPUT_DEBOUNCE_SAMPLES      3
#define INPUT_TASK_STACK_SIZE       2048
#define INPUT_TASK_PRIORITY         4
#define INPUT_ACTIVE_LEVEL          0

typedef struct {
    int slave_addr;
    int baud_rate;
    uart_word_length_t data_bits;
    uart_parity_t parity;
    uart_stop_bits_t stop_bits;
} modbus_config_t;

typedef struct {
    uart_port_t uart_num;
    int tx_gpio;
    int rx_gpio;
    int de_gpio;
    const char *task_name;
} rs485_port_t;

static const rs485_port_t s_rs485_ports[PLC_RS485_COUNT] = {
    {
        .uart_num = RS485_PORT_1_UART,
        .tx_gpio = RS485_PORT_1_TX_GPIO,
        .rx_gpio = RS485_PORT_1_RX_GPIO,
        .de_gpio = RS485_PORT_1_DE_GPIO,
        .task_name = "modbus_rs485_1",
    },
    {
        .uart_num = RS485_PORT_2_UART,
        .tx_gpio = RS485_PORT_2_TX_GPIO,
        .rx_gpio = RS485_PORT_2_RX_GPIO,
        .de_gpio = RS485_PORT_2_DE_GPIO,
        .task_name = "modbus_rs485_2",
    },
};

static modbus_config_t s_modbus_config[PLC_RS485_COUNT] = {
    {
        .slave_addr = MODBUS_SLAVE_ADDR,
        .baud_rate = RS485_BAUD_RATE,
        .data_bits = RS485_DATA_BITS,
        .parity = RS485_PARITY,
        .stop_bits = RS485_STOP_BITS,
    },
    {
        .slave_addr = MODBUS_SLAVE_ADDR,
        .baud_rate = RS485_BAUD_RATE,
        .data_bits = RS485_DATA_BITS,
        .parity = RS485_PARITY,
        .stop_bits = RS485_STOP_BITS,
    },
};

#define LED_COLOR_OFF               0, 0, 0
#define LED_COLOR_BOOT              0, 8, 0
#define LED_COLOR_WIFI_CONNECTING   8, 4, 0
#define LED_COLOR_ONLINE            0, 0, 10
#define LED_COLOR_OUTPUT_ACTIVITY    0, 12, 12
#define LED_COLOR_INPUT_ACTIVITY     12, 0, 12
#define LED_COLOR_ERROR             12, 0, 0

static const gpio_num_t s_output_gpios[PLC_OUTPUT_COUNT] = {
    GPIO_NUM_5,
    GPIO_NUM_18,
    GPIO_NUM_19,
    GPIO_NUM_21,    
};

static const gpio_num_t s_input_gpios[PLC_INPUT_COUNT] = {
    GPIO_NUM_35,
    GPIO_NUM_34,
    GPIO_NUM_39,
    GPIO_NUM_36,
};

static bool s_output_state[PLC_OUTPUT_COUNT];
static bool s_input_state[PLC_INPUT_COUNT];
static led_strip_handle_t s_status_strip;
static app_led_status_t s_status = APP_LED_STATUS_BOOT;

static esp_err_t status_led_set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    if (!s_status_strip) {
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(led_strip_set_pixel(s_status_strip, 0, r, g, b), TAG, "led set pixel failed");
    ESP_RETURN_ON_ERROR(led_strip_refresh(s_status_strip), TAG, "led refresh failed");
    return ESP_OK;
}

static esp_err_t status_led_apply(app_led_status_t status)
{
    s_status = status;
    switch (status) {
    case APP_LED_STATUS_BOOT:
        return status_led_set_rgb(LED_COLOR_BOOT);
    case APP_LED_STATUS_WIFI_CONNECTING:
        return status_led_set_rgb(LED_COLOR_WIFI_CONNECTING);
    case APP_LED_STATUS_ONLINE:
        return status_led_set_rgb(LED_COLOR_ONLINE);
    case APP_LED_STATUS_OUTPUT_ACTIVITY:
        return status_led_set_rgb(LED_COLOR_OUTPUT_ACTIVITY);
    case APP_LED_STATUS_INPUT_ACTIVITY:
        return status_led_set_rgb(LED_COLOR_INPUT_ACTIVITY);
    case APP_LED_STATUS_ERROR:
    default:
        return status_led_set_rgb(LED_COLOR_ERROR);
    }
}

static void init_status_led(void)
{
    const led_strip_config_t strip_config = {
        .strip_gpio_num = STATUS_LED_GPIO,
        .max_leds = STATUS_LED_COUNT,
        .led_model = LED_MODEL_WS2812,
        .flags.invert_out = false,
    };

    const led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 64,
        .flags.with_dma = false,
    };

    if (led_strip_new_rmt_device(&strip_config, &rmt_config, &s_status_strip) == ESP_OK) {
        led_strip_clear(s_status_strip);
        status_led_apply(APP_LED_STATUS_BOOT);
    } else {
        ESP_LOGW(TAG, "WS2812 init failed");
    }
}

/* Biến phục vụ nhấp nháy LED khi Reset */
static esp_timer_handle_t s_reset_blink_timer;
static uint8_t s_reset_r, s_reset_g, s_reset_b;
static bool s_reset_led_toggle = false;

static void reset_blink_timer_cb(void *arg)
{
    if (s_reset_led_toggle) {
        status_led_set_rgb(s_reset_r, s_reset_g, s_reset_b);
    } else {
        status_led_set_rgb(0, 0, 0);
    }
    s_reset_led_toggle = !s_reset_led_toggle;
}

static void reset_button_press_down_cb(void *arg, void *data)
{
    /* Bắt đầu nháy Xanh Lá */
    s_reset_r = 0; s_reset_g = STATUS_LED_BRIGHTNESS; s_reset_b = 0;
    s_reset_led_toggle = true;
    esp_timer_start_periodic(s_reset_blink_timer, RESET_BLINK_INTERVAL_MS * 1000);
}

static void reset_button_press_up_cb(void *arg, void *data)
{
    /* Dừng nháy và khôi phục trạng thái LED trước đó */
    esp_timer_stop(s_reset_blink_timer);
    status_led_apply(s_status);
}

static void reset_wifi_prompt_cb(void *arg, void *data)
{
    /* Chuyển sang nháy Vàng (đã đạt mốc WiFi Reset) */
    s_reset_r = STATUS_LED_BRIGHTNESS; 
    s_reset_g = STATUS_LED_BRIGHTNESS; 
    s_reset_b = 0;
}

static void reset_factory_prompt_cb(void *arg, void *data)
{
    /* Chuyển sang nháy Đỏ (đã đạt mốc Factory Reset) */
    s_reset_r = STATUS_LED_BRIGHTNESS; 
    s_reset_g = 0; 
    s_reset_b = 0;
}

static bool read_input_state(int index)
{
    return gpio_get_level(s_input_gpios[index]) == INPUT_ACTIVE_LEVEL;
}

static esp_err_t init_output_gpios(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 0,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    for (int i = 0; i < PLC_OUTPUT_COUNT; ++i) {
        io_conf.pin_bit_mask |= (1ULL << s_output_gpios[i]);
    }

    ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "output gpio config failed");

    for (int i = 0; i < PLC_OUTPUT_COUNT; ++i) {
        s_output_state[i] = false;
        gpio_set_level(s_output_gpios[i], 0);
    }

    return ESP_OK;
}

static esp_err_t init_input_gpios(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 0,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    for (int i = 0; i < PLC_INPUT_COUNT; ++i) {
        io_conf.pin_bit_mask |= (1ULL << s_input_gpios[i]);
    }

    ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "input gpio config failed");

    for (int i = 0; i < PLC_INPUT_COUNT; ++i) {
        s_input_state[i] = read_input_state(i);
    }

    return ESP_OK;
}

static void input_poll_task(void *arg)
{
    (void)arg;

    bool last_sample[PLC_INPUT_COUNT];
    uint8_t stable_samples[PLC_INPUT_COUNT];

    for (int i = 0; i < PLC_INPUT_COUNT; ++i) {
        last_sample[i] = s_input_state[i];
        stable_samples[i] = INPUT_DEBOUNCE_SAMPLES;
    }

    while (true) {
        for (int i = 0; i < PLC_INPUT_COUNT; ++i) {
            bool sample = read_input_state(i);

            if (sample == last_sample[i]) {
                if (stable_samples[i] < INPUT_DEBOUNCE_SAMPLES) {
                    stable_samples[i]++;
                }
            } else {
                last_sample[i] = sample;
                stable_samples[i] = 1;
            }

            if (stable_samples[i] >= INPUT_DEBOUNCE_SAMPLES && sample != s_input_state[i]) {
                s_input_state[i] = sample;
                ESP_LOGI(TAG, "Input %d -> %s", i + 1, sample ? "ON" : "OFF");
                app_driver_notify_input_change(i, sample);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(INPUT_POLL_INTERVAL_MS));
    }
}

static esp_err_t init_input_monitor(void)
{
    if (xTaskCreate(input_poll_task, "input_poll", INPUT_TASK_STACK_SIZE, NULL,
                    INPUT_TASK_PRIORITY, NULL) != pdPASS) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

static void init_reset_button(void)
{
    /* Khởi tạo Timer cho việc nhấp nháy */
    const esp_timer_create_args_t blink_timer_args = {
        .callback = &reset_blink_timer_cb,
        .name = "reset_blink"
    };
    esp_timer_create(&blink_timer_args, &s_reset_blink_timer);

    /* Tạo handle cho nút bấm tại GPIO 0 */
    button_handle_t btn_handle = app_reset_button_create(BOOT_BUTTON_GPIO, BOOT_BUTTON_ACTIVE_LEVEL);
    if (btn_handle) {
        /* Đăng ký logic Reset mặc định của RainMaker */
        app_reset_button_register(btn_handle, WIFI_RESET_TIMEOUT_S, FACTORY_RESET_TIMEOUT_S);

        /* Đăng ký thêm các sự kiện LED nhấp nháy */
        iot_button_register_cb(btn_handle, BUTTON_PRESS_DOWN, NULL, reset_button_press_down_cb, NULL);
        iot_button_register_cb(btn_handle, BUTTON_PRESS_UP, NULL, reset_button_press_up_cb, NULL);

        /* Mốc Wi-Fi Reset (Vàng) */
        button_event_args_t wifi_args = { .long_press.press_time = WIFI_RESET_TIMEOUT_S * 1000 };
        iot_button_register_cb(btn_handle, BUTTON_LONG_PRESS_START, &wifi_args, reset_wifi_prompt_cb, NULL);

        /* Mốc Factory Reset (Đỏ) */
        button_event_args_t factory_args = { .long_press.press_time = FACTORY_RESET_TIMEOUT_S * 1000 };
        iot_button_register_cb(btn_handle, BUTTON_LONG_PRESS_START, &factory_args, reset_factory_prompt_cb, NULL);
    }
}

static bool modbus_parse_data_bits(int val, uart_word_length_t *out)
{
    switch (val) {
    case 7:
        *out = UART_DATA_7_BITS;
        return true;
    case 8:
        *out = UART_DATA_8_BITS;
        return true;
    default:
        return false;
    }
}

static bool modbus_parse_parity(int val, uart_parity_t *out)
{
    switch (val) {
    case 0:
        *out = UART_PARITY_DISABLE;
        return true;
    case 1:
        *out = UART_PARITY_EVEN;
        return true;
    case 2:
        *out = UART_PARITY_ODD;
        return true;
    default:
        return false;
    }
}

static bool modbus_parse_stop_bits(int val, uart_stop_bits_t *out)
{
    switch (val) {
    case 1:
        *out = UART_STOP_BITS_1;
        return true;
    case 2:
        *out = UART_STOP_BITS_2;
        return true;
    default:
        return false;
    }
}

static esp_err_t modbus_apply_uart_config(int port)
{
    if (port < 0 || port >= PLC_RS485_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    uart_config_t uart_config = {
        .baud_rate = s_modbus_config[port].baud_rate,
        .data_bits = s_modbus_config[port].data_bits,
        .parity = s_modbus_config[port].parity,
        .stop_bits = s_modbus_config[port].stop_bits,
        .flow_ctrl = RS485_FLOW_CTRL,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(uart_param_config(s_rs485_ports[port].uart_num, &uart_config), TAG, "modbus uart param config failed");
    ESP_RETURN_ON_ERROR(uart_set_pin(s_rs485_ports[port].uart_num, s_rs485_ports[port].tx_gpio,
                                     s_rs485_ports[port].rx_gpio, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
                        TAG, "modbus uart set pin failed");
    return ESP_OK;
}

static uint16_t modbus_crc16(const uint8_t *buf, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= buf[i];
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

static void modbus_send_response(int port, const uint8_t *resp, size_t len)
{
    if (port < 0 || port >= PLC_RS485_COUNT) {
        return;
    }

    gpio_set_level(s_rs485_ports[port].de_gpio, 1);
    uart_write_bytes(s_rs485_ports[port].uart_num, (const char *)resp, len);
    uart_wait_tx_done(s_rs485_ports[port].uart_num, pdMS_TO_TICKS(100));
    gpio_set_level(s_rs485_ports[port].de_gpio, 0);
}

static void modbus_send_exception(int port, uint8_t slave_addr, uint8_t function_code, uint8_t exception_code)
{
    uint8_t resp[5];
    resp[0] = slave_addr;
    resp[1] = function_code | 0x80;
    resp[2] = exception_code;
    uint16_t crc = modbus_crc16(resp, 3);
    resp[3] = crc & 0xFF;
    resp[4] = crc >> 8;
    modbus_send_response(port, resp, sizeof(resp));
}

static bool modbus_frame_valid(const uint8_t *frame, size_t len)
{
    if (len < 5) {
        return false;
    }
    uint16_t received_crc = frame[len - 2] | (frame[len - 1] << 8);
    return received_crc == modbus_crc16(frame, len - 2);
}

static void modbus_handle_request(int port, uint8_t *frame, size_t len)
{
    if (port < 0 || port >= PLC_RS485_COUNT) {
        return;
    }
    if (!modbus_frame_valid(frame, len)) {
        return;
    }
    if (frame[0] != s_modbus_config[port].slave_addr) {
        return;
    }

    uint8_t function = frame[1];
    if (len < 8) {
        modbus_send_exception(port, s_modbus_config[port].slave_addr, function, 0x03);
        return;
    }

    uint16_t start_addr = (frame[2] << 8) | frame[3];
    uint16_t quantity = (frame[4] << 8) | frame[5];

    switch (function) {
    case 0x03: {
        if (start_addr + quantity > PLC_OUTPUT_COUNT || quantity == 0 || quantity > 125) {
            modbus_send_exception(port, s_modbus_config[port].slave_addr, function, 0x02);
            return;
        }
        size_t resp_len = 3 + quantity * 2 + 2;
        uint8_t resp[3 + 125 * 2 + 2];
        resp[0] = s_modbus_config[port].slave_addr;
        resp[1] = function;
        resp[2] = quantity * 2;
        for (uint16_t i = 0; i < quantity; ++i) {
            bool output_val = app_driver_get_output_state(start_addr + i);
            uint16_t reg_value = output_val ? 1 : 0;
            resp[3 + i * 2] = reg_value >> 8;
            resp[4 + i * 2] = reg_value & 0xFF;
        }
        uint16_t crc = modbus_crc16(resp, resp_len - 2);
        resp[resp_len - 2] = crc & 0xFF;
        resp[resp_len - 1] = crc >> 8;
        modbus_send_response(port, resp, resp_len);
        break;
    }
    case 0x04: {
        if (start_addr + quantity > PLC_INPUT_COUNT || quantity == 0 || quantity > 125) {
            modbus_send_exception(port, s_modbus_config[port].slave_addr, function, 0x02);
            return;
        }
        size_t resp_len = 3 + quantity * 2 + 2;
        uint8_t resp[3 + 125 * 2 + 2];
        resp[0] = s_modbus_config[port].slave_addr;
        resp[1] = function;
        resp[2] = quantity * 2;
        for (uint16_t i = 0; i < quantity; ++i) {
            bool input_val = app_driver_get_input_state(start_addr + i);
            uint16_t reg_value = input_val ? 1 : 0;
            resp[3 + i * 2] = reg_value >> 8;
            resp[4 + i * 2] = reg_value & 0xFF;
        }
        uint16_t crc = modbus_crc16(resp, resp_len - 2);
        resp[resp_len - 2] = crc & 0xFF;
        resp[resp_len - 1] = crc >> 8;
        modbus_send_response(port, resp, resp_len);
        break;
    }
    case 0x06: {
        if (start_addr >= PLC_OUTPUT_COUNT) {
            modbus_send_exception(port, s_modbus_config[port].slave_addr, function, 0x02);
            return;
        }
        uint16_t value = (frame[4] << 8) | frame[5];
        bool new_state = (value != 0);
        app_driver_set_output_state(start_addr, new_state);
        app_driver_notify_output_change(start_addr, new_state);
        uint8_t resp[8];
        memcpy(resp, frame, 6);
        uint16_t crc = modbus_crc16(resp, 6);
        resp[6] = crc & 0xFF;
        resp[7] = crc >> 8;
        modbus_send_response(port, resp, sizeof(resp));
        break;
    }
    case 0x10: {
        if (len < 9) {
            modbus_send_exception(port, s_modbus_config[port].slave_addr, function, 0x03);
            return;
        }
        uint8_t byte_count = frame[6];
        if (len != 9 + byte_count || byte_count != quantity * 2) {
            modbus_send_exception(port, s_modbus_config[port].slave_addr, function, 0x03);
            return;
        }
        if (start_addr + quantity > PLC_OUTPUT_COUNT || quantity == 0 || quantity > PLC_OUTPUT_COUNT) {
            modbus_send_exception(port, s_modbus_config[port].slave_addr, function, 0x02);
            return;
        }
        for (uint16_t i = 0; i < quantity; ++i) {
            uint16_t value = (frame[7 + i * 2] << 8) | frame[8 + i * 2];
            bool new_state = (value != 0);
            app_driver_set_output_state(start_addr + i, new_state);
            app_driver_notify_output_change(start_addr + i, new_state);
        }
        uint8_t resp[8];
        resp[0] = s_modbus_config[port].slave_addr;
        resp[1] = function;
        resp[2] = frame[2];
        resp[3] = frame[3];
        resp[4] = frame[4];
        resp[5] = frame[5];
        uint16_t crc = modbus_crc16(resp, 6);
        resp[6] = crc & 0xFF;
        resp[7] = crc >> 8;
        modbus_send_response(port, resp, sizeof(resp));
        break;
    }
    case 0x01: {
        if (start_addr + quantity > PLC_OUTPUT_COUNT || quantity == 0 || quantity > PLC_OUTPUT_COUNT) {
            modbus_send_exception(port, s_modbus_config[port].slave_addr, function, 0x02);
            return;
        }
        uint8_t byte_count = (quantity + 7) / 8;
        size_t resp_len = 3 + byte_count + 2;
        uint8_t resp[3 + 125 * 2 + 2];
        resp[0] = s_modbus_config[port].slave_addr;
        resp[1] = function;
        resp[2] = byte_count;
        memset(&resp[3], 0, byte_count);
        for (uint16_t i = 0; i < quantity; ++i) {
            bool coil = app_driver_get_output_state(start_addr + i);
            if (coil) {
                resp[3 + (i / 8)] |= (1 << (i % 8));
            }
        }
        uint16_t crc = modbus_crc16(resp, resp_len - 2);
        resp[resp_len - 2] = crc & 0xFF;
        resp[resp_len - 1] = crc >> 8;
        modbus_send_response(port, resp, resp_len);
        break;
    }
    case 0x02: {
        if (start_addr + quantity > PLC_INPUT_COUNT || quantity == 0 || quantity > PLC_INPUT_COUNT) {
            modbus_send_exception(port, s_modbus_config[port].slave_addr, function, 0x02);
            return;
        }
        uint8_t byte_count = (quantity + 7) / 8;
        size_t resp_len = 3 + byte_count + 2;
        uint8_t resp[3 + PLC_INPUT_COUNT + 2];
        resp[0] = s_modbus_config[port].slave_addr;
        resp[1] = function;
        resp[2] = byte_count;
        memset(&resp[3], 0, byte_count);
        for (uint16_t i = 0; i < quantity; ++i) {
            bool input = app_driver_get_input_state(start_addr + i);
            if (input) {
                resp[3 + (i / 8)] |= (1 << (i % 8));
            }
        }
        uint16_t crc = modbus_crc16(resp, resp_len - 2);
        resp[resp_len - 2] = crc & 0xFF;
        resp[resp_len - 1] = crc >> 8;
        modbus_send_response(port, resp, resp_len);
        break;
    }
    default:
        modbus_send_exception(port, s_modbus_config[port].slave_addr, function, 0x01);
        break;
    }
}

static void modbus_task(void *arg)
{
    int port = (int)(intptr_t)arg;
    if (port < 0 || port >= PLC_RS485_COUNT) {
        vTaskDelete(NULL);
        return;
    }

    uint8_t buffer[256];
    while (true) {
        int len = uart_read_bytes(s_rs485_ports[port].uart_num, buffer, sizeof(buffer),
                                  pdMS_TO_TICKS(MODBUS_RX_TIMEOUT_MS));
        if (len > 0) {
            modbus_handle_request(port, buffer, len);
        }
    }
}

static esp_err_t init_rs485_uart(int port)
{
    if (port < 0 || port >= PLC_RS485_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    uart_config_t uart_config = {
        .baud_rate = s_modbus_config[port].baud_rate,
        .data_bits = s_modbus_config[port].data_bits,
        .parity = s_modbus_config[port].parity,
        .stop_bits = s_modbus_config[port].stop_bits,
        .flow_ctrl = RS485_FLOW_CTRL,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_RETURN_ON_ERROR(uart_param_config(s_rs485_ports[port].uart_num, &uart_config), TAG, "uart param config failed");
    ESP_RETURN_ON_ERROR(uart_set_pin(s_rs485_ports[port].uart_num, s_rs485_ports[port].tx_gpio,
                                     s_rs485_ports[port].rx_gpio, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
                        TAG, "uart set pin failed");
    ESP_RETURN_ON_ERROR(uart_driver_install(s_rs485_ports[port].uart_num, 1024, 1024, 0, NULL, 0),
                        TAG, "uart driver install failed");
    return ESP_OK;
}

static esp_err_t init_rs485(void)
{
    for (int port = 0; port < PLC_RS485_COUNT; port++) {
        ESP_RETURN_ON_ERROR(init_rs485_uart(port), TAG, "rs485 uart init failed");
    }

    uint64_t de_pin_mask = 0;
    for (int port = 0; port < PLC_RS485_COUNT; port++) {
        de_pin_mask |= (1ULL << s_rs485_ports[port].de_gpio);
    }

    gpio_config_t de_conf = {
        .pin_bit_mask = de_pin_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_RETURN_ON_ERROR(gpio_config(&de_conf), TAG, "de gpio config failed");
    for (int port = 0; port < PLC_RS485_COUNT; port++) {
        gpio_set_level(s_rs485_ports[port].de_gpio, 0);
    }

    return ESP_OK;
}

static esp_err_t init_modbus(void)
{
    for (int port = 0; port < PLC_RS485_COUNT; port++) {
        if (xTaskCreate(modbus_task, s_rs485_ports[port].task_name, MODBUS_TASK_STACK_SIZE,
                        (void *)(intptr_t)port, MODBUS_TASK_PRIORITY, NULL) != pdPASS) {
            return ESP_FAIL;
        }
    }
    return ESP_OK;
}

esp_err_t app_driver_init_all(void)
{
    ESP_RETURN_ON_ERROR(init_output_gpios(), TAG, "output init failed");
    ESP_RETURN_ON_ERROR(init_input_gpios(), TAG, "input init failed");
    ESP_RETURN_ON_ERROR(init_input_monitor(), TAG, "input monitor init failed");
    ESP_RETURN_ON_ERROR(init_rs485(), TAG, "rs485 init failed");
    ESP_RETURN_ON_ERROR(init_modbus(), TAG, "modbus init failed");
    init_reset_button();
    init_status_led();
    status_led_apply(APP_LED_STATUS_WIFI_CONNECTING);
    return ESP_OK;
}

void app_driver_init(void)
{
    if (app_driver_init_all() != ESP_OK) {
        status_led_apply(APP_LED_STATUS_ERROR);
    }
}

void app_driver_set_output_state(int index, bool state)
{
    if (index < 0 || index >= PLC_OUTPUT_COUNT) {
        return;
    }
    s_output_state[index] = state;
    gpio_set_level(s_output_gpios[index], state ? 1 : 0);
    status_led_apply(APP_LED_STATUS_OUTPUT_ACTIVITY);
}

bool app_driver_get_output_state(int index)
{
    if (index < 0 || index >= PLC_OUTPUT_COUNT) {
        return false;
    }
    return s_output_state[index];
}

bool app_driver_get_input_state(int index)
{
    if (index < 0 || index >= PLC_INPUT_COUNT) {
        return false;
    }
    return s_input_state[index];
}

esp_err_t app_driver_rs485_write(int port, const uint8_t *data, size_t len)
{
    if (port < 0 || port >= PLC_RS485_COUNT || !data || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    gpio_set_level(s_rs485_ports[port].de_gpio, 1);
    int written = uart_write_bytes(s_rs485_ports[port].uart_num, (const char *)data, len);
    uart_wait_tx_done(s_rs485_ports[port].uart_num, pdMS_TO_TICKS(100));
    gpio_set_level(s_rs485_ports[port].de_gpio, 0);

    if (written < 0 || (size_t)written != len) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

void app_driver_update_modbus_config(int port, int slave_addr, int baud_rate, int data_bits, int parity, int stop_bits)
{
    if (port < 0 || port >= PLC_RS485_COUNT) {
        return;
    }

    if (slave_addr >= 1 && slave_addr <= 247) {
        s_modbus_config[port].slave_addr = slave_addr;
    }
    if (baud_rate >= 1200 && baud_rate <= 115200) {
        s_modbus_config[port].baud_rate = baud_rate;
    }
    uart_word_length_t parsed_bits;
    if (modbus_parse_data_bits(data_bits, &parsed_bits)) {
        s_modbus_config[port].data_bits = parsed_bits;
    }
    uart_parity_t parsed_parity;
    if (modbus_parse_parity(parity, &parsed_parity)) {
        s_modbus_config[port].parity = parsed_parity;
    }
    uart_stop_bits_t parsed_stop;
    if (modbus_parse_stop_bits(stop_bits, &parsed_stop)) {
        s_modbus_config[port].stop_bits = parsed_stop;
    }
    (void)modbus_apply_uart_config(port);
}

void app_driver_set_status(app_led_status_t status)
{
    (void)status_led_apply(status);
}

void app_driver_pulse_status(app_led_status_t status, uint32_t duration_ms)
{
    app_led_status_t prev = s_status;
    (void)status_led_apply(status);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    (void)status_led_apply(prev);
}
