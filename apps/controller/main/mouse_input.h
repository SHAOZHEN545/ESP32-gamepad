#pragma once

#include <stdint.h>

#include "analog_sticks.h"

void mouse_input_start(void);
stick_position_t mouse_input_read_stick(void);
