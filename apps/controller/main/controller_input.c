#include "controller_input.h"

#include <stdint.h>

#include "analog_sticks.h"
#include "board_config.h"
#include "button_expander.h"
#include "core_switch_imu.h"
#include "driver/gpio.h"
#include "mouse_input.h"
#include "touch_triggers.h"

static hoja_core_t s_core = HOJA_CORE_NS;
static bool s_mouse_enabled = true;

static int16_t clamp_i16(int32_t value)
{
    if (value > INT16_MAX) return INT16_MAX;
    if (value < INT16_MIN) return INT16_MIN;
    return (int16_t)value;
}

static void update_switch_mouse_gyro(bool zr_pressed)
{
    /* Drain motion while the aim modifier is up so it cannot carry into the
     * next ZR press. The CH9350 has no accelerometer, so accel remains zero. */
    mouse_motion_t motion = mouse_input_read_motion();
    ns_imu_sample_s sample = {0};

    if (s_mouse_enabled && zr_pressed && ns_imu_is_enabled()) {
        /* Flip both axes based on the first Switch hardware aim test. */
        sample.gz = clamp_i16(-(int32_t)motion.x * BOARD_MOUSE_GYRO_YAW_RAW_PER_DELTA);
        sample.gx = clamp_i16((int32_t)motion.y * BOARD_MOUSE_GYRO_PITCH_RAW_PER_DELTA);
    }

    ns_imu_set_sample(&sample);
}

void controller_input_init(void)
{
    touch_triggers_init();
    button_expander_init();
    analog_sticks_init();

    gpio_config_t config = {
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = BOARD_GPIO_INPUT_MASK,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&config));
}

void controller_input_set_mode(hoja_core_t core, bool mouse_enabled)
{
    s_core = core;
    s_mouse_enabled = mouse_enabled;
    if (mouse_enabled) {
        mouse_input_start();
    }
}

static void read_buttons(void)
{
    uint32_t gpio = REG_READ(GPIO_IN_REG);
    button_expander_state_t expander = button_expander_read();

    hoja_button_data.dpad_down = !util_getbit(gpio, BOARD_GPIO_BTN_DPAD_DOWN);
    hoja_button_data.dpad_left = !util_getbit(gpio, BOARD_GPIO_BTN_DPAD_LEFT);
    hoja_button_data.dpad_right = !util_getbit(gpio, BOARD_GPIO_BTN_DPAD_RIGHT);
    hoja_button_data.dpad_up = !util_getbit(gpio, BOARD_GPIO_BTN_DPAD_UP);
    hoja_button_data.button_right = !util_getbit(gpio, BOARD_GPIO_BTN_A);
    hoja_button_data.button_down = !util_getbit(gpio, BOARD_GPIO_BTN_B);
    hoja_button_data.button_up = !util_getbit(gpio, BOARD_GPIO_BTN_X);
    hoja_button_data.button_left = !util_getbit(gpio, BOARD_GPIO_BTN_Y);

    bool zl = touch_trigger_zl_pressed();
    bool zr = touch_trigger_zr_pressed() ||
        (s_core == HOJA_CORE_NS && s_mouse_enabled && mouse_input_right_pressed());
    hoja_button_data.trigger_zl = zl;
    hoja_button_data.trigger_zr = zr;
    hoja_analog_data.lt_a = zl ? 255 : 0;
    hoja_analog_data.rt_a = zr ? 255 : 0;

    hoja_button_data.trigger_l = button_expander_l_pressed(expander);
    hoja_button_data.trigger_r = button_expander_r_pressed(expander);
    hoja_button_data.button_stick_left = button_expander_left_stick_pressed(expander);
    hoja_button_data.button_stick_right = button_expander_right_stick_pressed(expander);
    hoja_button_data.button_select = button_expander_select_pressed(expander);
    hoja_button_data.button_start = button_expander_start_pressed(expander);
    hoja_button_data.button_capture = button_expander_capture_pressed(expander);
    hoja_button_data.button_home = button_expander_home_pressed(expander);
    hoja_button_data.button_sleep = button_expander_select_pressed(expander);
}

static void read_analog_sticks(void)
{
    bool switch_axis_direction = s_core == HOJA_CORE_NS;
    stick_position_t left = analog_sticks_read_left(switch_axis_direction);
    stick_position_t right = analog_sticks_read_right(switch_axis_direction);
    bool zr_pressed = touch_trigger_zr_pressed() ||
        (s_core == HOJA_CORE_NS && s_mouse_enabled && mouse_input_right_pressed());

    if (s_core == HOJA_CORE_NS) {
        update_switch_mouse_gyro(zr_pressed);
    } else if (zr_pressed && s_mouse_enabled) {
        right = mouse_input_read_stick();
    }

    hoja_analog_data.ls_x = left.x;
    hoja_analog_data.ls_y = left.y;
    hoja_analog_data.rs_x = right.x;
    hoja_analog_data.rs_y = right.y;
}

static void handle_event(hoja_event_type_t type, uint8_t event, uint8_t parameter)
{
    (void)type;
    (void)event;
    (void)parameter;
}

void controller_input_register_callbacks(void)
{
    hoja_register_button_callback(read_buttons);
    hoja_register_analog_callback(read_analog_sticks);
    hoja_register_event_callback(handle_event);
}
