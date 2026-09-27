#include "mouse_input.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <limits.h>

#include "board_config.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"

#define UART_PORT UART_NUM_2
#define BUFFER_SIZE 1024
#define STICK_CENTER (0x740 + 0x80)
#define STICK_MIN 0xFA
#define STICK_MAX 0xF47
#define MOUSE_SENSITIVITY 200.0f
#define MOUSE_IDLE_TIMEOUT_MS 40
#define CH9350_MOUSE_REPORT_SIZE 7
#define CH9350_MOUSE_RIGHT_BUTTON 0x02

static const char *TAG = "mouse_input";
static int32_t s_delta_x;
static int32_t s_delta_y;
static uint8_t s_buttons;
static uint32_t s_last_move_ms;
static portMUX_TYPE s_motion_lock = portMUX_INITIALIZER_UNLOCKED;

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
    uint8_t frame[CH9350_MOUSE_REPORT_SIZE] = {0};
    size_t frame_length = 0;
    while (true) {
        int length = uart_read_bytes(UART_PORT, data, BUFFER_SIZE, 0);
        bool found_delta = false;

        for (int i = 0; i < length; ++i) {
            if (i > 0 && (i % 256) == 0) {
                vTaskDelay(1);
            }
            uint8_t byte = data[i];
            if (frame_length == 0 && byte != 0x57) continue;
            if (frame_length == 1 && byte != 0xAB) {
                frame_length = byte == 0x57 ? 1 : 0;
                continue;
            }
            if (frame_length == 2 && byte != 0x02) {
                frame_length = byte == 0x57 ? 1 : 0;
                continue;
            }

            frame[frame_length++] = byte;
            if (frame_length == CH9350_MOUSE_REPORT_SIZE) {
                int8_t dx = (int8_t)frame[4];
                int8_t dy = (int8_t)frame[5];
                portENTER_CRITICAL(&s_motion_lock);
                s_delta_x += dx;
                s_delta_y += dy;
                s_buttons = frame[3];
                portEXIT_CRITICAL(&s_motion_lock);
                found_delta = found_delta || dx != 0 || dy != 0;
                frame_length = 0;
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
    mouse_motion_t motion = mouse_input_read_motion();
    int delta_x = motion.x;
    int delta_y = motion.y;

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

mouse_motion_t mouse_input_read_motion(void)
{
    int32_t delta_x;
    int32_t delta_y;

    portENTER_CRITICAL(&s_motion_lock);
    delta_x = s_delta_x;
    delta_y = s_delta_y;
    s_delta_x = 0;
    s_delta_y = 0;
    portEXIT_CRITICAL(&s_motion_lock);

    if (delta_x > INT16_MAX) delta_x = INT16_MAX;
    if (delta_x < INT16_MIN) delta_x = INT16_MIN;
    if (delta_y > INT16_MAX) delta_y = INT16_MAX;
    if (delta_y < INT16_MIN) delta_y = INT16_MIN;

    return (mouse_motion_t) { .x = (int16_t)delta_x, .y = (int16_t)delta_y };
}

bool mouse_input_right_pressed(void)
{
    bool pressed;
    portENTER_CRITICAL(&s_motion_lock);
    pressed = (s_buttons & CH9350_MOUSE_RIGHT_BUTTON) != 0;
    portEXIT_CRITICAL(&s_motion_lock);
    return pressed;
}
