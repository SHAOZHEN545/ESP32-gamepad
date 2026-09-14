#include "touch_triggers.h"

#include <stdint.h>

#include "board_config.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "touch_triggers";
static uint16_t s_zr_baseline;
static uint16_t s_zl_baseline;
static bool s_zr_calibrated;
static bool s_zl_calibrated;

static uint16_t calibrate_touch_pad(touch_pad_t pad, const char *name)
{
    uint32_t sum = 0;
    uint16_t value = 0;
    const int sample_count = 10;

    ESP_LOGI(TAG, "Calibrating %s; do not touch the trigger", name);
    for (int i = 0; i < sample_count; ++i) {
        ESP_ERROR_CHECK(touch_pad_read_raw_data(pad, &value));
        sum += value;
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    value = (uint16_t)(sum / sample_count);
    ESP_LOGI(TAG, "%s baseline: %u", name, value);
    return value;
}

void touch_triggers_init(void)
{
    ESP_ERROR_CHECK(touch_pad_init());
    ESP_ERROR_CHECK(touch_pad_config(BOARD_TOUCH_ZR, 0));
    ESP_ERROR_CHECK(touch_pad_config(BOARD_TOUCH_ZL, 0));
    ESP_ERROR_CHECK(touch_pad_filter_start(10));

    s_zr_baseline = calibrate_touch_pad(BOARD_TOUCH_ZR, "ZR");
    s_zr_calibrated = true;
    s_zl_baseline = calibrate_touch_pad(BOARD_TOUCH_ZL, "ZL");
    s_zl_calibrated = true;
}

static bool is_pressed(touch_pad_t pad, uint16_t baseline, bool calibrated)
{
    if (!calibrated) {
        return false;
    }

    uint16_t value = 0;
    ESP_ERROR_CHECK(touch_pad_read_raw_data(pad, &value));
    return value < (baseline * BOARD_TOUCH_THRESHOLD_RATIO);
}

bool touch_trigger_zl_pressed(void)
{
    return is_pressed(BOARD_TOUCH_ZL, s_zl_baseline, s_zl_calibrated);
}

bool touch_trigger_zr_pressed(void)
{
    return is_pressed(BOARD_TOUCH_ZR, s_zr_baseline, s_zr_calibrated);
}
