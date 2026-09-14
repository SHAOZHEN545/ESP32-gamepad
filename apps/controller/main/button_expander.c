#include "button_expander.h"

#include <stdbool.h>

#include "board_config.h"
#include "driver/i2c.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"

#define REG_INPUT_0 0x00
#define REG_INPUT_1 0x01
#define REG_OUTPUT_0 0x02
#define REG_OUTPUT_1 0x03
#define REG_CONFIG_0 0x06
#define REG_CONFIG_1 0x07

#define PORT0_L_MASK (1U << 0)
#define PORT0_LEFT_STICK_MASK (1U << 1)
#define PORT0_SELECT_MASK (1U << 2)
#define PORT0_CAPTURE_MASK (1U << 3)
#define PORT1_R_MASK (1U << 7)
#define PORT1_RIGHT_STICK_MASK (1U << 6)
#define PORT1_START_MASK (1U << 5)
#define PORT1_HOME_MASK (1U << 4)

static esp_err_t write_register(uint8_t reg, uint8_t value)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BOARD_TCA9555_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);
    esp_err_t result = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return result;
}

static esp_err_t read_register(uint8_t reg, uint8_t *value)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BOARD_TCA9555_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BOARD_TCA9555_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, value, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t result = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return result;
}

void button_expander_init(void)
{
    i2c_config_t config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = BOARD_I2C_SDA_GPIO,
        .scl_io_num = BOARD_I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = BOARD_I2C_FREQUENCY_HZ,
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, config.mode, 0, 0, 0));
    ESP_ERROR_CHECK(write_register(REG_CONFIG_0, 0xFF));
    ESP_ERROR_CHECK(write_register(REG_CONFIG_1, 0xFF));
    ESP_ERROR_CHECK(write_register(REG_OUTPUT_0, 0xFF));
    ESP_ERROR_CHECK(write_register(REG_OUTPUT_1, 0xFF));
}

button_expander_state_t button_expander_read(void)
{
    button_expander_state_t state = {0};
    ESP_ERROR_CHECK(read_register(REG_INPUT_0, &state.port0));
    ESP_ERROR_CHECK(read_register(REG_INPUT_1, &state.port1));
    return state;
}

bool button_expander_l_pressed(button_expander_state_t s) { return !(s.port0 & PORT0_L_MASK); }
bool button_expander_r_pressed(button_expander_state_t s) { return !(s.port1 & PORT1_R_MASK); }
bool button_expander_left_stick_pressed(button_expander_state_t s) { return !(s.port0 & PORT0_LEFT_STICK_MASK); }
bool button_expander_right_stick_pressed(button_expander_state_t s) { return !(s.port1 & PORT1_RIGHT_STICK_MASK); }
bool button_expander_select_pressed(button_expander_state_t s) { return !(s.port0 & PORT0_SELECT_MASK); }
bool button_expander_start_pressed(button_expander_state_t s) { return !(s.port1 & PORT1_START_MASK); }
bool button_expander_capture_pressed(button_expander_state_t s) { return !(s.port0 & PORT0_CAPTURE_MASK); }
bool button_expander_home_pressed(button_expander_state_t s) { return !(s.port1 & PORT1_HOME_MASK); }
