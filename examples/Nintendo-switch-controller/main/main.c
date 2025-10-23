#include <stdio.h>
#include "hoja_includes.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/touch_pad.h"

//============================================= 触摸板配置 =============================================
#define TOUCH_PAD_ZR TOUCH_PAD_NUM0 // ZR用TOUCH0
#define TOUCH_PAD_ZL TOUCH_PAD_NUM4
static const char *TOUCH_TAG = "TOUCH";
static uint16_t touch_base_value_zr = 0; // ZR (Touch 0) 的基准值
static uint16_t touch_base_value_zl = 0; // ZL (Touch 4) 的基准值
static bool touch_calibrated_zr = false;
static bool touch_calibrated_zl = false;

// 初始化触摸板
void custom_touch_init()
{
    // 初始化触摸板
    ESP_ERROR_CHECK(touch_pad_init());
    
    // 配置触摸板（不设置阈值）
    ESP_ERROR_CHECK(touch_pad_config(TOUCH_PAD_ZR, 0)); // 配置ZR
    ESP_ERROR_CHECK(touch_pad_config(TOUCH_PAD_ZL, 0)); // 配置ZL
    
    // 设置滤波（可选，根据你的需求）
    ESP_ERROR_CHECK(touch_pad_filter_start(10));  // 滤波周期为10ms
    
    // --- 校准 ZR (Touch 0) ---
    uint16_t touch_value;
    uint32_t sum = 0;
    const int calibration_samples = 10;
    
    ESP_LOGI(TOUCH_TAG, "开始触摸板ZR校准，请勿触摸铝板...");
    for (int i = 0; i < calibration_samples; i++) {
        ESP_ERROR_CHECK(touch_pad_read_raw_data(TOUCH_PAD_ZR, &touch_value));
        sum += touch_value;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    touch_base_value_zr = sum / calibration_samples;
    touch_calibrated_zr = true;
    ESP_LOGI(TOUCH_TAG, "触摸板(ZR)校准完成，基准值: %d", touch_base_value_zr);
    
    // --- 校准 ZL (Touch 4) ---
    sum = 0; // 重置sum
    ESP_LOGI(TOUCH_TAG, "开始触摸板(ZL)校准，请勿触摸...");
    for (int i = 0; i < calibration_samples; i++) {
        ESP_ERROR_CHECK(touch_pad_read_raw_data(TOUCH_PAD_ZL, &touch_value));
        sum += touch_value;
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    touch_base_value_zl = sum / calibration_samples;
    touch_calibrated_zl = true;

    ESP_LOGI(TOUCH_TAG, "触摸板(ZL)校准完成，基准值: %d", touch_base_value_zl);
}

// 检测ZR触摸状态
bool is_touched_zr()
{
    if (!touch_calibrated_zr) return false;
    
    uint16_t touch_value;
    ESP_ERROR_CHECK(touch_pad_read_raw_data(TOUCH_PAD_ZR, &touch_value));
    
    // 当触摸值比基准值低30%时判定为触摸
    // 你可以调整这个阈值（0.7表示30%下降，0.6表示40%下降等）
    const float touch_threshold = 0.7f;
    
    return touch_value < (touch_base_value_zr * touch_threshold);
}

// 检测ZL触摸状态
bool is_touched_zl()
{
    if (!touch_calibrated_zl) return false;

    uint16_t touch_value;
    ESP_ERROR_CHECK(touch_pad_read_raw_data(TOUCH_PAD_ZL, &touch_value));

    // 当触摸值比基准值低30%时判定为触摸
    const float touch_threshold = 0.7f;

    return touch_value < (touch_base_value_zl * touch_threshold);
}

//============================================= TCA9555相关配置 =============================================
// TCA9555 按钮引脚定义
// Port0
#define TCA9555_BTN_L_PIN          0  // Port0.0
#define TCA9555_BTN_STICK_L_PIN    1  // Port0.1
#define TCA9555_BTN_SELECT_PIN     2  // Port0.2
#define TCA9555_BTN_CAPTURE_PIN    3  // Port0.3

// Port1
#define TCA9555_BTN_R_PIN          7  // Port1.7
#define TCA9555_BTN_STICK_R_PIN    6  // Port1.6
#define TCA9555_BTN_START_PIN      5  // Port1.5
#define TCA9555_BTN_HOME_PIN       4  // Port1.4

// 创建对应的位掩码
// Port0掩码
#define TCA9555_BTN_L_MASK         (1 << TCA9555_BTN_L_PIN)
#define TCA9555_BTN_STICK_L_MASK   (1 << TCA9555_BTN_STICK_L_PIN)
#define TCA9555_BTN_SELECT_MASK    (1 << TCA9555_BTN_SELECT_PIN)
#define TCA9555_BTN_CAPTURE_MASK   (1 << TCA9555_BTN_CAPTURE_PIN)

// Port1掩码
#define TCA9555_BTN_R_MASK         (1 << TCA9555_BTN_R_PIN)
#define TCA9555_BTN_STICK_R_MASK   (1 << TCA9555_BTN_STICK_R_PIN)
#define TCA9555_BTN_START_MASK     (1 << TCA9555_BTN_START_PIN)
#define TCA9555_BTN_HOME_MASK      (1 << TCA9555_BTN_HOME_PIN)

// TCA9555 相关定义
#define TCA9555_ADDR 0x20
#define REG_INPUT_0 0x00
#define REG_INPUT_1 0x01
#define REG_OUTPUT_0 0x02
#define REG_OUTPUT_1 0x03
#define REG_CONFIG_0 0x06
#define REG_CONFIG_1 0x07

#define I2C_MASTER_PORT I2C_NUM_0
#define I2C_MASTER_SDA_GPIO 21
#define I2C_MASTER_SCL_GPIO 22
#define I2C_MASTER_FREQ_HZ 100000

static void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_GPIO,
        .scl_io_num = I2C_MASTER_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0));
}

static esp_err_t write_register(uint8_t reg, uint8_t value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TCA9555_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t read_register(uint8_t reg, uint8_t *value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TCA9555_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TCA9555_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, value, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

//============================================= 按钮和模拟摇杆配置 =============================================
// 按钮GPIO定义
#define GPIO_BTN_A          GPIO_NUM_19
#define GPIO_BTN_B          GPIO_NUM_18
#define GPIO_BTN_X          GPIO_NUM_17
#define GPIO_BTN_Y          GPIO_NUM_23
#define GPIO_BTN_DPAD_U     GPIO_NUM_25 
#define GPIO_BTN_DPAD_L     GPIO_NUM_27
#define GPIO_BTN_DPAD_D     GPIO_NUM_26
#define GPIO_BTN_DPAD_R     GPIO_NUM_14

// GPIO输入引脚掩码
#define GPIO_INPUT_PIN_MASK     ( (1ULL << GPIO_BTN_A)|(1ULL << GPIO_BTN_B)|(1ULL << GPIO_BTN_X)|(1ULL << GPIO_BTN_Y)|(1ULL << GPIO_BTN_DPAD_U)|(1ULL << GPIO_BTN_DPAD_L)|(1ULL << GPIO_BTN_DPAD_D)|(1ULL << GPIO_BTN_DPAD_R))

// 摇杆通道定义
typedef enum {
    CH_LS_X = ADC_CHANNEL_4,  // GPIO32
    CH_LS_Y = ADC_CHANNEL_5,  // GPIO33
    CH_RS_X = ADC_CHANNEL_6,  // GPIO34
    CH_RS_Y = ADC_CHANNEL_7   // GPIO35
} joystick_channel_t;

#define JOYSTICK_CHANNELS_COUNT 4

// 通道映射函数
static int channel_to_index(joystick_channel_t channel)
{
    switch(channel) {
        case ADC_CHANNEL_4: return 0; // CH_LS_X
        case ADC_CHANNEL_5: return 1; // CH_LS_Y
        case ADC_CHANNEL_6: return 2; // CH_RS_X
        case ADC_CHANNEL_7: return 3; // CH_RS_Y
        default: return 0;
    }
}
static const adc_channel_t joystick_channels[JOYSTICK_CHANNELS_COUNT] = {
    CH_LS_X, CH_LS_Y, CH_RS_X, CH_RS_Y
};

static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t adc_cali_handle = NULL;

// 摇杆校准配置结构体
typedef struct {
    uint16_t phys_min;     // 实测最小值
    uint16_t phys_center;  // 实测中心值
    uint16_t phys_max;     // 实测最大值
    uint16_t deadzone;     // 死区大小
    bool     invert;       // 是否需要反转
    uint16_t lib_min;      // 库期望的最小值
    uint16_t lib_center;   // 库期望的中心值
    uint16_t lib_max;      // 库期望的最大值
} joystick_calib_t;

// 左摇杆X轴 - 灵敏模式(默认)
static const joystick_calib_t calib_left_x_sensitive = {
    .phys_min = 1435,      .phys_center = 1835, .phys_max = 2235,
    .deadzone = 100,    .invert = true,
    .lib_min = 0xFA,    .lib_center = 0x740, .lib_max = 0xF47
};


// 左摇杆Y轴 - 灵敏模式(默认)
static const joystick_calib_t calib_left_y_sensitive = {
    .phys_min = 1335,      .phys_center = 1735, .phys_max = 2135,
    .deadzone = 100,    .invert = true,
    .lib_min = 0xFA,    .lib_center = 0x740, .lib_max = 0xF47
};

// 右摇杆X轴 - 灵敏模式(默认)
static const joystick_calib_t calib_right_x_sensitive = {
    .phys_min = 1485,      .phys_center = 1835, .phys_max = 2285,
    .deadzone = 100,    .invert = true,
    .lib_min = 0xFA,    .lib_center = 0x740 + 0x80, .lib_max = 0xF47
};

// 右摇杆Y轴 - 灵敏模式(默认)
static const joystick_calib_t calib_right_y_sensitive = {
    .phys_min = 1495,      .phys_center = 1845, .phys_max = 2195,
    .deadzone = 100,    .invert = false,
    .lib_min = 0xFA,    .lib_center = 0x740 + 0x80, .lib_max = 0xF47
};

//============================================= 摇杆状态变量 =============================================

// 摇杆值映射函数
static uint16_t map_joystick_value(uint16_t raw, const joystick_calib_t* calib)
{
    // 限制输入范围
    if (raw < calib->phys_min) raw = calib->phys_min;
    if (raw > calib->phys_max) raw = calib->phys_max;

    // 死区处理
    int16_t diff = raw - calib->phys_center;
    uint16_t abs_diff = abs(diff);
    
    if (abs_diff <= calib->deadzone) {
        return calib->lib_center;
    }

    // 计算比例因子
    float ratio;
    if (diff > 0) {
        ratio = (float)(raw - calib->phys_center) / (calib->phys_max - calib->phys_center);
    } else {
        ratio = (float)(calib->phys_center - raw) / (calib->phys_center - calib->phys_min);
    }

    uint16_t result;
    
    // 应用反转逻辑
    if (calib->invert) {
        if (diff > 0) {
            result = calib->lib_center - ratio * (calib->lib_center - calib->lib_min);
        } else {
            result = calib->lib_center + ratio * (calib->lib_max - calib->lib_center);
        }
    } else {
        if (diff > 0) {
            result = calib->lib_center + ratio * (calib->lib_max - calib->lib_center);
        } else {
            result = calib->lib_center - ratio * (calib->lib_center - calib->lib_min);
        }
    }

    // 确保结果在有效范围内
    if (result < calib->lib_min) result = calib->lib_min;
    else if (result > calib->lib_max) result = calib->lib_max;

    return result;
}


// ADC配置
#define ADC_UNIT ADC_UNIT_1
#define ADC_ATTEN ADC_ATTEN_DB_11
#define ADC_BITWIDTH ADC_BITWIDTH_12
// ADC校准初始化
static bool adc_calibration_init()
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT,
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH,
    };
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
    if (ret == ESP_OK) {
        calibrated = true;
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = ADC_UNIT,
            .atten = ADC_ATTEN,
            .bitwidth = ADC_BITWIDTH,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    adc_cali_handle = handle;
    return calibrated;
}

// 初始化ADC
static void adc_init(void)
{
    // ADC单次采样配置
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));

    // 配置每个通道
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH,
    };

    for (int i = 0; i < JOYSTICK_CHANNELS_COUNT; i++) {
        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, joystick_channels[i], &config));
    }

    // 初始化校准
    if (!adc_calibration_init()) {
        printf("使用原始ADC值（未校准）\n");
    }
}

// ADC读取函数
static int read_adc_raw(joystick_channel_t channel)
{
    static int last_raw_values[JOYSTICK_CHANNELS_COUNT] = {
        1835,  // CH_LS_X
        1735,  // CH_LS_Y
        1835,  // CH_RS_X
        1845   // CH_RS_Y
    };
    
    int index = channel_to_index(channel);
    int raw = 0;
    esp_err_t ret = adc_oneshot_read(adc1_handle, channel, &raw);
    
    if (ret != ESP_OK) {
        ESP_LOGW("ADC", "Channel %d read failed, using last value: %d", 
                channel, last_raw_values[index]);
        return last_raw_values[index];
    }
    
    last_raw_values[index] = raw;
    return raw;
}

//============================================= 必要的回调函数定义 =============================================
// 按钮扫描回调函数
void local_button_cb()
{
    uint32_t register_read_low = REG_READ(GPIO_IN_REG);
    uint8_t port0_state, port1_state;
    
    // 读取TCA9555的状态
    ESP_ERROR_CHECK(read_register(REG_INPUT_0, &port0_state));
    ESP_ERROR_CHECK(read_register(REG_INPUT_1, &port1_state));

    // 设置按钮状态
    hoja_button_data.dpad_down      = !util_getbit(register_read_low, GPIO_BTN_DPAD_D);
    hoja_button_data.dpad_left      = !util_getbit(register_read_low, GPIO_BTN_DPAD_L);
    hoja_button_data.dpad_right     = !util_getbit(register_read_low, GPIO_BTN_DPAD_R);
    hoja_button_data.dpad_up        = !util_getbit(register_read_low, GPIO_BTN_DPAD_U);

    hoja_button_data.button_right   = !util_getbit(register_read_low, GPIO_BTN_A);
    hoja_button_data.button_down    = !util_getbit(register_read_low, GPIO_BTN_B);
    hoja_button_data.button_up    = !util_getbit(register_read_low, GPIO_BTN_X);
    hoja_button_data.button_left    = !util_getbit(register_read_low, GPIO_BTN_Y);
    
    // 使用触摸板检测ZR触发
    hoja_button_data.trigger_zl     = is_touched_zl();
    hoja_button_data.trigger_zr     = is_touched_zr();
    
    // 从TCA9555读取的按钮 (使用宏定义)
    // Port0按钮
    hoja_button_data.trigger_l      = !(port0_state & TCA9555_BTN_L_MASK);
    hoja_button_data.button_stick_left = !(port0_state & TCA9555_BTN_STICK_L_MASK);
    hoja_button_data.button_select  = !(port0_state & TCA9555_BTN_SELECT_MASK);
    hoja_button_data.button_capture = !(port0_state & TCA9555_BTN_CAPTURE_MASK);
    
    // Port1按钮
    hoja_button_data.trigger_r      = !(port1_state & TCA9555_BTN_R_MASK);
    hoja_button_data.button_stick_right = !(port1_state & TCA9555_BTN_STICK_R_MASK);
    hoja_button_data.button_start   = !(port1_state & TCA9555_BTN_START_MASK);
    hoja_button_data.button_home    = !(port1_state & TCA9555_BTN_HOME_MASK);
    
    // button_sleep保持使用select键
    hoja_button_data.button_sleep   = !(port0_state & TCA9555_BTN_SELECT_MASK);
}

// 摇杆数据回调函数
void local_analog_cb() 
{      
    // 读取原始ADC值
    int ls_x_raw = read_adc_raw(CH_LS_X);
    int ls_y_raw = read_adc_raw(CH_LS_Y);
    int rs_x_raw = read_adc_raw(CH_RS_X);
    int rs_y_raw = read_adc_raw(CH_RS_Y);
    
    const joystick_calib_t* left_x_calib = &calib_left_x_sensitive;
    const joystick_calib_t* left_y_calib = &calib_left_y_sensitive;
    const joystick_calib_t* right_x_calib = &calib_right_x_sensitive;
    const joystick_calib_t* right_y_calib = &calib_right_y_sensitive;
    
    // 应用校准映射
    hoja_analog_data.ls_x = map_joystick_value(ls_x_raw, left_x_calib);
    hoja_analog_data.ls_y = map_joystick_value(ls_y_raw, left_y_calib);
    hoja_analog_data.rs_x = map_joystick_value(rs_x_raw, right_x_calib);
    hoja_analog_data.rs_y = map_joystick_value(rs_y_raw, right_y_calib);
}

// 事件回调函数
void local_event_cb(hoja_event_type_t type, uint8_t evt, uint8_t param)
{
    if (type == HOJA_EVT_SYSTEM && evt == HEVT_API_SHUTDOWN) {
        // 休眠事件处理
    }
}

//================================================= 主函数 ====================================================

void app_main(void)
{
    const char* TAG = "app_main";

    // 初始化触摸板
    custom_touch_init();

    // 初始化 I2C 和 TCA9555
    i2c_master_init();
    // 配置 TCA9555 的所有端口为输入模式
    ESP_ERROR_CHECK(write_register(REG_CONFIG_0, 0xFF));
    ESP_ERROR_CHECK(write_register(REG_CONFIG_1, 0xFF));
    // 启用所有端口的内部上拉电阻
    ESP_ERROR_CHECK(write_register(REG_OUTPUT_0, 0xFF));
    ESP_ERROR_CHECK(write_register(REG_OUTPUT_1, 0xFF));

    // 初始化ADC
    adc_init();

    // 设置GPIO配置
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = GPIO_INPUT_PIN_MASK,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io_conf);

    // 注册回调函数
    hoja_register_button_callback(local_button_cb);
    hoja_register_analog_callback(local_analog_cb);
    hoja_register_event_callback(local_event_cb);

    // 初始化HOJA
    hoja_err_t err = hoja_init();
    if (err != HOJA_OK) {   
        ESP_LOGE(TAG, "Failed to initialize HOJA.");
    }
    else {
        // 设置并启动Nintendo Switch核心
        hoja_set_core(HOJA_CORE_NS);
        hoja_start_core();
    }
}