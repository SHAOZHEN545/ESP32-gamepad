#pragma once

#include "driver/gpio.h"
#include "driver/touch_pad.h"
#include "esp_adc/adc_oneshot.h"

// Direct GPIO buttons (active low).
#define BOARD_GPIO_BTN_A GPIO_NUM_19
#define BOARD_GPIO_BTN_B GPIO_NUM_18
#define BOARD_GPIO_BTN_X GPIO_NUM_17
#define BOARD_GPIO_BTN_Y GPIO_NUM_23
#define BOARD_GPIO_BTN_DPAD_UP GPIO_NUM_25
#define BOARD_GPIO_BTN_DPAD_LEFT GPIO_NUM_27
#define BOARD_GPIO_BTN_DPAD_DOWN GPIO_NUM_26
#define BOARD_GPIO_BTN_DPAD_RIGHT GPIO_NUM_14

#define BOARD_GPIO_INPUT_MASK \
    ((1ULL << BOARD_GPIO_BTN_A) | (1ULL << BOARD_GPIO_BTN_B) | \
     (1ULL << BOARD_GPIO_BTN_X) | (1ULL << BOARD_GPIO_BTN_Y) | \
     (1ULL << BOARD_GPIO_BTN_DPAD_UP) | (1ULL << BOARD_GPIO_BTN_DPAD_LEFT) | \
     (1ULL << BOARD_GPIO_BTN_DPAD_DOWN) | (1ULL << BOARD_GPIO_BTN_DPAD_RIGHT))

// ESP32 capacitive inputs used as digital ZL/ZR triggers.
#define BOARD_TOUCH_ZR TOUCH_PAD_NUM0
#define BOARD_TOUCH_ZL TOUCH_PAD_NUM4
#define BOARD_TOUCH_THRESHOLD_RATIO 0.78f

// TCA9555 button expander.
#define BOARD_I2C_SDA_GPIO GPIO_NUM_21
#define BOARD_I2C_SCL_GPIO GPIO_NUM_22
#define BOARD_I2C_FREQUENCY_HZ 100000
#define BOARD_TCA9555_ADDRESS 0x20

// CH9350L USB-host mouse receiver.
#define BOARD_MOUSE_UART_RX_GPIO GPIO_NUM_16
#define BOARD_MOUSE_UART_BAUD_RATE 115200
/* Switch mouse-gyro sensitivity. Mouse X controls yaw (horizontal aim), and
 * mouse Y controls pitch (vertical aim). Lower values slow the corresponding
 * axis. The previous common value was 80; 160 was too fast in hardware. */
#define BOARD_MOUSE_GYRO_YAW_RAW_PER_DELTA 55
#define BOARD_MOUSE_GYRO_PITCH_RAW_PER_DELTA 150

// Analog joystick channels on ADC1.
#define BOARD_ADC_LS_X ADC_CHANNEL_4 // GPIO32
#define BOARD_ADC_LS_Y ADC_CHANNEL_5 // GPIO33
#define BOARD_ADC_RS_X ADC_CHANNEL_6 // GPIO34
#define BOARD_ADC_RS_Y ADC_CHANNEL_7 // GPIO35
