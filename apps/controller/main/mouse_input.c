#include "mouse_input.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "board_config.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define UART_PORT UART_NUM_2
#define BUFFER_SIZE 1024
#define STICK_CENTER (0x740 + 0x80)
#define STICK_MIN 0xFA
#define STICK_MAX 0xF47
#define MOUSE_SENSITIVITY 200.0f
#define MOUSE_IDLE_TIMEOUT_MS 40

static const char *TAG = "mouse_input";
static volatile int16_t s_delta_x;
static volatile int16_t s_delta_y;
static uint32_t s_last_move_ms;

static uint16_t clamp_stick(int value)
{
    if (value < STICK_MIN) return STICK_MIN;
    if (value > STICK_MAX) return STICK_MAX;
    return (uint16_t)value;
}

static void reader_task(void *parameters)
{
    uint8_t *data = malloc(BUFFER_SIZE);
    if (data == NULL) {
        ESP_LOGE(TAG, "Could not allocate UART buffer");
        vTaskDelete(NULL);
        return;
    }

    bool mouse_was_moving = false;
    while (true) {
        int length = uart_read_bytes(UART_PORT, data, BUFFER_SIZE, 0);
        bool found_delta = false;

        for (int i = 0; i < length; ++i) {
            if (i > 0 && (i % 256) == 0) {
                vTaskDelay(1);
            }
            if (data[i] == 0x57 && i + 6 < length &&
                data[i + 1] == 0xAB && data[i + 2] == 0x02) {
                int8_t dx = (int8_t)data[i + 4];
                int8_t dy = (int8_t)data[i + 5];
                s_delta_x += dx;
                s_delta_y += dy;
                found_delta = found_delta || dx != 0 || dy != 0;
                i += 6;
            }
        }

        if (found_delta) {
            mouse_was_moving = true;
        } else if (length == 0 && mouse_was_moving) {
            s_last_move_ms = esp_log_timestamp() - (MOUSE_IDLE_TIMEOUT_MS + 1);
            mouse_was_moving = false;
        }
        vTaskDelay(1);
    }
}

void mouse_input_start(void)
{
    uart_config_t config = {
        .baud_rate = BOARD_MOUSE_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, BUFFER_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &config));
    ESP_ERROR_CHECK(uart_set_pin(
        UART_PORT, UART_PIN_NO_CHANGE, BOARD_MOUSE_UART_RX_GPIO,
        UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    xTaskCreate(reader_task, "ch9350_reader", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "CH9350 mouse input started");
}

stick_position_t mouse_input_read_stick(void)
{
    static stick_position_t position = {STICK_CENTER, STICK_CENTER};

    int delta_x = s_delta_x;
    s_delta_x = 0;
    int delta_y = s_delta_y;
    s_delta_y = 0;

    if (delta_x != 0 || delta_y != 0) {
        s_last_move_ms = esp_log_timestamp();
        position.x = clamp_stick(STICK_CENTER + (int)(delta_x * MOUSE_SENSITIVITY));
        position.y = clamp_stick(STICK_CENTER - (int)(delta_y * MOUSE_SENSITIVITY));
    } else if ((esp_log_timestamp() - s_last_move_ms) > MOUSE_IDLE_TIMEOUT_MS) {
        position.x = STICK_CENTER;
        position.y = STICK_CENTER;
    }

    return position;
}
