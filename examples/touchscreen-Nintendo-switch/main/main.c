#include <stdio.h>
#include <stdlib.h>
#include "hoja_includes.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

// ===================== 1. 硬件定义 =====================
#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define GPIO_TP_RST                 4
#define GPIO_TP_INT                 5 

#define I2C_MASTER_NUM              0
// ★★★ 关键修改：降速到 100kHz 以适应无外部上拉电阻的环境 ★★★
#define I2C_MASTER_FREQ_HZ          100000 
#define CST816_ADDR                 0x15   

// ===================== 2. 按钮定义 (已移除冲突引脚) =====================
#define GPIO_BTN_A          GPIO_NUM_19
#define GPIO_BTN_B          GPIO_NUM_18
#define GPIO_BTN_DPAD_U     GPIO_NUM_25 
#define GPIO_BTN_DPAD_L     GPIO_NUM_27
#define GPIO_BTN_DPAD_D     GPIO_NUM_26
#define GPIO_BTN_DPAD_R     GPIO_NUM_14
#define GPIO_BTN_ZL         GPIO_NUM_13
#define GPIO_BTN_ZR         GPIO_NUM_23
#define GPIO_BTN_START      GPIO_NUM_12

#define GPIO_INPUT_PIN_MASK     ( (1ULL << GPIO_BTN_A)|(1ULL << GPIO_BTN_B)|(1ULL << GPIO_BTN_DPAD_U)|(1ULL << GPIO_BTN_DPAD_L)|(1ULL << GPIO_BTN_DPAD_D)|(1ULL << GPIO_BTN_DPAD_R)|(1ULL << GPIO_BTN_ZL)|(1ULL << GPIO_BTN_ZR)|(1ULL << GPIO_BTN_START))

// ===================== 3. 全局变量 =====================
typedef struct {
    uint16_t raw_x; 
    uint16_t raw_y; 
    bool touched;   
} touch_state_t;

volatile touch_state_t g_touch_state = { .raw_x = 120, .raw_y = 120, .touched = false };

typedef struct {
    uint16_t phys_min; uint16_t phys_center; uint16_t phys_max;
    uint16_t deadzone; bool invert;
    uint16_t lib_min; uint16_t lib_center; uint16_t lib_max;
} joystick_calib_t;

// static const joystick_calib_t touch_calib_x = {
//     .phys_min = 50,    .phys_center = 120, .phys_max = 190, // 收缩范围
//     .deadzone = 15,    .invert = false,
//     .lib_min = 0xFA,   .lib_center = 0x740, .lib_max = 0xF47
// };

// static const joystick_calib_t touch_calib_y = {
//     .phys_min = 50,    .phys_center = 120, .phys_max = 190, // 收缩范围
//     .deadzone = 15,    .invert = true, 
//     .lib_min = 0xFA,   .lib_center = 0x740, .lib_max = 0xF47
// };

// 右摇杆 X轴 (控制视角左右)
static const joystick_calib_t touch_calib_rs_x = {
    .phys_min = 50,    .phys_center = 120, .phys_max = 190, // 保持舒服的物理范围
    .deadzone = 15,    .invert = false,                     // 左划看左，右划看右
    .lib_min = 0xFA,   .lib_center = 0x740 + 0x80, .lib_max = 0xF47 // ★★★ 注意中心偏移 ★★★
};

// 右摇杆 Y轴 (控制视角上下)
// 注意：Switch 视角通常也是反转的（上划抬头），如果进游戏发现反了，就把 invert 改成 false
static const joystick_calib_t touch_calib_rs_y = {
    .phys_min = 50,    .phys_center = 120, .phys_max = 190, // 保持舒服的物理范围
    .deadzone = 15,    .invert = true,                      // 上划(Y减小) -> 输出大值 -> 抬头
    .lib_min = 0xFA,   .lib_center = 0x740 + 0x80, .lib_max = 0xF47 // ★★★ 注意中心偏移 ★★★
};

// ===================== 4. 辅助函数 =====================
static uint16_t map_joystick_value(uint16_t raw, const joystick_calib_t* calib)
{
    if (raw < calib->phys_min) raw = calib->phys_min;
    if (raw > calib->phys_max) raw = calib->phys_max;

    int16_t diff = raw - calib->phys_center;
    uint16_t abs_diff = abs(diff);
    
    if (abs_diff <= calib->deadzone) return calib->lib_center;

    float ratio;
    if (diff > 0) ratio = (float)(raw - calib->phys_center) / (calib->phys_max - calib->phys_center);
    else ratio = (float)(calib->phys_center - raw) / (calib->phys_center - calib->phys_min);

    uint16_t result;
    if (calib->invert) {
        if (diff > 0) result = calib->lib_center - ratio * (calib->lib_center - calib->lib_min);
        else result = calib->lib_center + ratio * (calib->lib_max - calib->lib_center);
    } else {
        if (diff > 0) result = calib->lib_center + ratio * (calib->lib_max - calib->lib_center);
        else result = calib->lib_center - ratio * (calib->lib_center - calib->lib_min);
    }

    if (result < calib->lib_min) result = calib->lib_min;
    else if (result > calib->lib_max) result = calib->lib_max;

    return result;
}

static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE, // 必须开启！
        .scl_pullup_en = GPIO_PULLUP_ENABLE, // 必须开启！
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static void touch_reset(void)
{
    gpio_config_t io_conf = { .pin_bit_mask = (1ULL << GPIO_TP_RST), .mode = GPIO_MODE_OUTPUT, .pull_up_en = 1 };
    gpio_config(&io_conf);
    gpio_set_level(GPIO_TP_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(GPIO_TP_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
}

// ★★★ 新增：I2C 扫描函数 (用于启动时诊断) ★★★
static void i2c_scanner(void) {
    ESP_LOGW("I2C_SCAN", "Scanning I2C Bus at 100kHz...");
    int devices_found = 0;
    for (int i = 1; i < 127; i++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        if (i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(10)) == ESP_OK) {
            ESP_LOGW("I2C_SCAN", "Found device at: 0x%02X", i);
            devices_found++;
        }
        i2c_cmd_link_delete(cmd);
    }
    if (devices_found == 0) ESP_LOGE("I2C_SCAN", "NO DEVICES FOUND! Check wiring!");
    else ESP_LOGW("I2C_SCAN", "Scan complete.");
}

// ===================== 5. 任务 =====================
void touch_task(void *pvParameters)
{
    uint8_t data[7];
    int no_touch_counter = 0;
    
    // 初始化
    g_touch_state.raw_x = 120;
    g_touch_state.raw_y = 120;
    g_touch_state.touched = false;

    // 调试打印分频
    int log_divider = 0;

    while (1) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (CST816_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, 0x00, true);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (CST816_ADDR << 1) | I2C_MASTER_READ, true);
        i2c_master_read(cmd, data, 7, I2C_MASTER_LAST_NACK);
        i2c_master_stop(cmd);
        
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            // ★★★ 核心修改：只信任 Data[2] (触摸点数) ★★★
            // 只有当芯片明确说 "有点在屏幕上" (Points > 0) 时，我们才更新坐标
            bool valid_touch_event = (data[2] > 0);

            if (valid_touch_event) {
                // 有触摸：重置松手计数器
                no_touch_counter = 0;
                
                uint16_t x = ((data[3] & 0x0F) << 8) | data[4];
                uint16_t y = ((data[5] & 0x0F) << 8) | data[6];

                g_touch_state.raw_x = x;
                g_touch_state.raw_y = y;
                g_touch_state.touched = true;

                if (log_divider++ > 5) {
                    // 打印看看现在的坐标
                    ESP_LOGI("TOUCH", "Active! (%d, %d)", x, y);
                    log_divider = 0;
                }
            } 
            else {
                // 没触摸 (Points == 0)：开始累计计数器
                // 注意：此时我们【不更新】raw_x/raw_y，防止读取到残留的 0 或旧数据
                no_touch_counter++;
                
                // 连续 3 帧 (约60ms) 没摸到，才判定为真正松手
                if (no_touch_counter > 3) {
                    g_touch_state.touched = false;
                    
                    // ★★★ 强制回中 ★★★
                    g_touch_state.raw_x = 120; 
                    g_touch_state.raw_y = 120;
                    
                    no_touch_counter = 4; // 锁定计数器
                }
            }
        } 
        
        vTaskDelay(pdMS_TO_TICKS(20)); 
    }
}

// ===================== 6. 回调 =====================
void local_button_cb() {
    uint32_t register_read_low = REG_READ(GPIO_IN_REG);
    hoja_button_data.dpad_down      = !util_getbit(register_read_low, GPIO_BTN_DPAD_D);
    hoja_button_data.dpad_left      = !util_getbit(register_read_low, GPIO_BTN_DPAD_L);
    hoja_button_data.dpad_right     = !util_getbit(register_read_low, GPIO_BTN_DPAD_R);
    hoja_button_data.dpad_up        = !util_getbit(register_read_low, GPIO_BTN_DPAD_U);
    hoja_button_data.button_right   = !util_getbit(register_read_low, GPIO_BTN_A);
    hoja_button_data.button_down    = !util_getbit(register_read_low, GPIO_BTN_B);
    hoja_button_data.trigger_zl     = !util_getbit(register_read_low, GPIO_BTN_ZL);
    hoja_button_data.trigger_zr     = !util_getbit(register_read_low, GPIO_BTN_ZR);
    hoja_button_data.button_start   = !util_getbit(register_read_low, GPIO_BTN_START);
}

void local_analog_cb()
{
    // --- 1. 左摇杆 (固定锁死在标准中心) ---
    // 你之前测试左摇杆用的标准中心是 0x740，这里保持不动
    hoja_analog_data.ls_x = 0x740;
    hoja_analog_data.ls_y = 0x740;

    // --- 2. 右摇杆 (由触摸屏接管) ---
    uint16_t fx, fy;

    if (g_touch_state.touched) {
        // 使用新的 RS 校准参数 (含 0x80 偏移)
        fx = map_joystick_value(g_touch_state.raw_x, &touch_calib_rs_x);
        fy = map_joystick_value(g_touch_state.raw_y, &touch_calib_rs_y);
    } 
    else {
        // 松手回中，回到右摇杆的特殊中心
        fx = touch_calib_rs_x.lib_center; // 0x740 + 0x80
        fy = touch_calib_rs_y.lib_center; // 0x740 + 0x80
    }

    hoja_analog_data.rs_x = fx;
    hoja_analog_data.rs_y = fy;

    // 调试打印 (可选，确认中心值是否正确)
    static int log_c = 0;
    if (g_touch_state.touched && log_c++ > 200) {
        ESP_LOGI("RS_MAP", "RS X: 0x%03X, RS Y: 0x%03X (Center is approx 0x7C0)", fx, fy);
        log_c = 0;
    }
}

void local_event_cb(hoja_event_type_t type, uint8_t evt, uint8_t param) {
    if (type == HOJA_EVT_SYSTEM && evt == HEVT_API_SHUTDOWN) {}
}

// ===================== 7. Main =====================
void app_main(void)
{
    // 
 
    // 上拉电阻电路示意：SDA/SCL 需要通过电阻接 3.3V
    // 因为我们没有外部电阻，我们依赖 ESP32 内部弱上拉，并降低速度到 100kHz。

    ESP_ERROR_CHECK(i2c_master_init());
    touch_reset();

    // ★★★ 启动前先自检，让你心里有底 ★★★
    i2c_scanner();

    xTaskCreate(touch_task, "touch_task", 4096, NULL, 5, NULL);

    gpio_config_t io_conf = {0};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_MASK; 
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    hoja_register_button_callback(local_button_cb);
    hoja_register_analog_callback(local_analog_cb);
    hoja_register_event_callback(local_event_cb);

    hoja_err_t err = hoja_init();
    if (err == HOJA_OK) {
        hoja_set_core(HOJA_CORE_BT_XINPUT);
        hoja_start_core();
    }
}