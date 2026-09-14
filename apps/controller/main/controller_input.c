#include "controller_input.h"

#include "analog_sticks.h"
#include "board_config.h"
#include "button_expander.h"
#include "driver/gpio.h"
#include "mouse_input.h"
#include "touch_triggers.h"

static hoja_core_t s_core = HOJA_CORE_NS;
static bool s_mouse_enabled = true;

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
    bool zr = touch_trigger_zr_pressed();
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
    stick_position_t right;

    if (touch_trigger_zr_pressed() && s_mouse_enabled) {
        right = mouse_input_read_stick();
    } else {
        right = analog_sticks_read_right(switch_axis_direction);
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
