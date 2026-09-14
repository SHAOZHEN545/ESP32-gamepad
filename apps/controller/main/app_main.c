#include <stdbool.h>

#include "board_config.h"
#include "controller_input.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hoja_includes.h"

static const char *TAG = "app_main";

typedef struct {
    hoja_core_t core;
    bool mouse_enabled;
} startup_mode_t;

static startup_mode_t read_startup_mode(void)
{
    startup_mode_t mode = {
        .core = HOJA_CORE_NS,
        .mouse_enabled = true,
    };

    int button_a = gpio_get_level(BOARD_GPIO_BTN_A);
    int button_b = gpio_get_level(BOARD_GPIO_BTN_B);
    if (button_a != 0 && button_b != 0) {
        return mode;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
    button_a = gpio_get_level(BOARD_GPIO_BTN_A);
    button_b = gpio_get_level(BOARD_GPIO_BTN_B);
    mode.mouse_enabled = button_a != 0;
    mode.core = button_b == 0 ? HOJA_CORE_BT_XINPUT : HOJA_CORE_NS;

    ESP_LOGI(TAG, "Mouse input: %s", mode.mouse_enabled ? "enabled" : "disabled");
    ESP_LOGI(TAG, "Controller mode: %s", mode.core == HOJA_CORE_NS ? "Switch" : "XInput");

    while (gpio_get_level(BOARD_GPIO_BTN_A) == 0 ||
           gpio_get_level(BOARD_GPIO_BTN_B) == 0) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return mode;
}

void app_main(void)
{
    controller_input_init();
    startup_mode_t mode = read_startup_mode();
    controller_input_set_mode(mode.core, mode.mouse_enabled);
    controller_input_register_callbacks();

    hoja_err_t result = hoja_init();
    if (result != HOJA_OK) {
        ESP_LOGE(TAG, "Failed to initialize HOJA");
        return;
    }

    hoja_set_core(mode.core);
    hoja_start_core();
}
