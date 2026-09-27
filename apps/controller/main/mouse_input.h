#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "analog_sticks.h"

typedef struct {
    int16_t x;
    int16_t y;
} mouse_motion_t;

void mouse_input_start(void);
stick_position_t mouse_input_read_stick(void);
mouse_motion_t mouse_input_read_motion(void);
bool mouse_input_right_pressed(void);
