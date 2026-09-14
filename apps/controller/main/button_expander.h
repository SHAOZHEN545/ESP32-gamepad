#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint8_t port0;
    uint8_t port1;
} button_expander_state_t;

void button_expander_init(void);
button_expander_state_t button_expander_read(void);

bool button_expander_l_pressed(button_expander_state_t state);
bool button_expander_r_pressed(button_expander_state_t state);
bool button_expander_left_stick_pressed(button_expander_state_t state);
bool button_expander_right_stick_pressed(button_expander_state_t state);
bool button_expander_select_pressed(button_expander_state_t state);
bool button_expander_start_pressed(button_expander_state_t state);
bool button_expander_capture_pressed(button_expander_state_t state);
bool button_expander_home_pressed(button_expander_state_t state);
