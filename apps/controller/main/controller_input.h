#pragma once

#include <stdbool.h>

#include "hoja_includes.h"

void controller_input_init(void);
void controller_input_set_mode(hoja_core_t core, bool mouse_enabled);
void controller_input_register_callbacks(void);
