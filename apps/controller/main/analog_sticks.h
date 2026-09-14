#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint16_t x;
    uint16_t y;
} stick_position_t;

void analog_sticks_init(void);
stick_position_t analog_sticks_read_left(bool invert_y_for_switch);
stick_position_t analog_sticks_read_right(bool invert_y_for_switch);
