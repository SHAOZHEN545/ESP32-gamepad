#include "analog_sticks.h"

#include <stdlib.h>

#include "board_config.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_log.h"

#define ADC_UNIT ADC_UNIT_1
#define ADC_ATTEN ADC_ATTEN_DB_11
#define ADC_BITWIDTH ADC_BITWIDTH_12
#define JOYSTICK_CHANNEL_COUNT 4

typedef struct {
    uint16_t phys_min;
    uint16_t phys_center;
    uint16_t phys_max;
    uint16_t deadzone;
    bool invert;
    uint16_t output_min;
    uint16_t output_center;
    uint16_t output_max;
} joystick_calibration_t;

static const adc_channel_t CHANNELS[JOYSTICK_CHANNEL_COUNT] = {
    BOARD_ADC_LS_X, BOARD_ADC_LS_Y, BOARD_ADC_RS_X, BOARD_ADC_RS_Y,
};

static const joystick_calibration_t LEFT_X = {
    1435, 1835, 2235, 100, true, 0xFA, 0x740, 0xF47,
};
static const joystick_calibration_t LEFT_Y = {
    1335, 1735, 2135, 100, false, 0xFA, 0x740, 0xF47,
};
static const joystick_calibration_t RIGHT_X = {
    1485, 1835, 2285, 100, true, 0xFA, 0x740 + 0x80, 0xF47,
};
static const joystick_calibration_t RIGHT_Y = {
    1495, 1845, 2195, 100, true, 0xFA, 0x740 + 0x80, 0xF47,
};

static adc_oneshot_unit_handle_t s_adc_handle;
static adc_cali_handle_t s_calibration_handle;

static int channel_index(adc_channel_t channel)
{
    switch (channel) {
        case BOARD_ADC_LS_X: return 0;
        case BOARD_ADC_LS_Y: return 1;
        case BOARD_ADC_RS_X: return 2;
        case BOARD_ADC_RS_Y: return 3;
        default: return 0;
    }
}

static bool init_adc_calibration(void)
{
    esp_err_t result = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t config = {
        .unit_id = ADC_UNIT,
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH,
    };
    result = adc_cali_create_scheme_curve_fitting(&config, &s_calibration_handle);
    calibrated = result == ESP_OK;
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        adc_cali_line_fitting_config_t config = {
            .unit_id = ADC_UNIT,
            .atten = ADC_ATTEN,
            .bitwidth = ADC_BITWIDTH,
        };
        result = adc_cali_create_scheme_line_fitting(&config, &s_calibration_handle);
        calibrated = result == ESP_OK;
    }
#endif

    return calibrated;
}

void analog_sticks_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_config, &s_adc_handle));

    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH,
    };
    for (int i = 0; i < JOYSTICK_CHANNEL_COUNT; ++i) {
        ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, CHANNELS[i], &channel_config));
    }

    if (!init_adc_calibration()) {
        ESP_LOGW("analog_sticks", "ADC calibration unavailable; using raw readings");
    }
}

static int read_raw(adc_channel_t channel)
{
    static int last_values[JOYSTICK_CHANNEL_COUNT] = {1835, 1735, 1835, 1845};
    int index = channel_index(channel);
    int value = 0;
    esp_err_t result = adc_oneshot_read(s_adc_handle, channel, &value);
    if (result != ESP_OK) {
        ESP_LOGW("analog_sticks", "ADC channel %d failed; using %d", channel, last_values[index]);
        return last_values[index];
    }
    last_values[index] = value;
    return value;
}

static uint16_t map_value(uint16_t raw, const joystick_calibration_t *calibration)
{
    if (raw < calibration->phys_min) raw = calibration->phys_min;
    if (raw > calibration->phys_max) raw = calibration->phys_max;

    int16_t difference = raw - calibration->phys_center;
    if (abs(difference) <= calibration->deadzone) {
        return calibration->output_center;
    }

    float ratio = difference > 0
        ? (float)(raw - calibration->phys_center) / (calibration->phys_max - calibration->phys_center)
        : (float)(calibration->phys_center - raw) / (calibration->phys_center - calibration->phys_min);

    int result;
    if (calibration->invert) {
        result = difference > 0
            ? calibration->output_center - ratio * (calibration->output_center - calibration->output_min)
            : calibration->output_center + ratio * (calibration->output_max - calibration->output_center);
    } else {
        result = difference > 0
            ? calibration->output_center + ratio * (calibration->output_max - calibration->output_center)
            : calibration->output_center - ratio * (calibration->output_center - calibration->output_min);
    }

    if (result < calibration->output_min) return calibration->output_min;
    if (result > calibration->output_max) return calibration->output_max;
    return (uint16_t)result;
}

static joystick_calibration_t with_optional_inversion(
    const joystick_calibration_t *source, bool invert)
{
    joystick_calibration_t result = *source;
    if (invert) {
        result.invert = !result.invert;
    }
    return result;
}

stick_position_t analog_sticks_read_left(bool invert_y_for_switch)
{
    joystick_calibration_t y = with_optional_inversion(&LEFT_Y, invert_y_for_switch);
    return (stick_position_t) {
        .x = map_value(read_raw(BOARD_ADC_LS_X), &LEFT_X),
        .y = map_value(read_raw(BOARD_ADC_LS_Y), &y),
    };
}

stick_position_t analog_sticks_read_right(bool invert_y_for_switch)
{
    joystick_calibration_t y = with_optional_inversion(&RIGHT_Y, invert_y_for_switch);
    return (stick_position_t) {
        .x = map_value(read_raw(BOARD_ADC_RS_X), &RIGHT_X),
        .y = map_value(read_raw(BOARD_ADC_RS_Y), &y),
    };
}
