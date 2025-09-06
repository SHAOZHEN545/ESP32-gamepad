#include <stdio.h>
#include "hoja_includes.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

/**
 *  This demo is designed to demonstrate how straightforward it is to get a controller
 *  up and running with minimal programming effort. Some foreknowledge of ESP-IDF is
 *  useful to resolve issues with compilation. The CMake system is not as automated
 *  as other IDEs such as Arduino, but the performance gains are worth your time!
 * 
 *  This gamepad is a standard layout similar to the SNES buttons. No analog sticks
 *  used in this example (More to come on that eventually)
*/

// We must define our GPIO. Doing our own
// defines in this way makes the code easier to read and change later on.
#define GPIO_BTN_A          GPIO_NUM_19
#define GPIO_BTN_B          GPIO_NUM_18
#define GPIO_BTN_X          GPIO_NUM_17
#define GPIO_BTN_Y          GPIO_NUM_16
#define GPIO_BTN_DPAD_U     GPIO_NUM_25 
#define GPIO_BTN_DPAD_L     GPIO_NUM_27
#define GPIO_BTN_DPAD_D     GPIO_NUM_26
#define GPIO_BTN_DPAD_R     GPIO_NUM_14
#define GPIO_BTN_ZL         GPIO_NUM_13
#define GPIO_BTN_ZR         GPIO_NUM_23
#define GPIO_BTN_START      GPIO_NUM_5
#define GPIO_BTN_SELECT     GPIO_NUM_12

// ADC配置
#define ADC_UNIT ADC_UNIT_1
#define ADC_ATTEN ADC_ATTEN_DB_11
#define ADC_BITWIDTH ADC_BITWIDTH_12

// 摇杆通道定义
typedef enum {
    CH_LS_X = 0,  // 左摇杆X (ADC_CHANNEL_4/GPIO32)
    CH_LS_Y,      // 左摇杆Y (ADC_CHANNEL_5/GPIO33)
    CH_RS_X,      // 右摇杆X (ADC_CHANNEL_6/GPIO34)
    CH_RS_Y,      // 右摇杆Y (ADC_CHANNEL_7/GPIO35)
    JOYSTICK_CHANNELS_COUNT
} joystick_channel_t;

static const adc_channel_t joystick_channels[JOYSTICK_CHANNELS_COUNT] = {
    ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_6, ADC_CHANNEL_7
};

static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t adc_cali_handle = NULL;

// ==================== 摇杆校准配置 ====================
typedef struct {
    uint16_t phys_min;     // 实测最小值 (a)
    uint16_t phys_center;  // 实测中心值 (m)
    uint16_t phys_max;     // 实测最大值 (b)
    uint16_t deadzone;     // 死区大小
    bool     invert;       // 是否需要反转
} joystick_calib_t;

// 左摇杆校准配置 (根据你的实测数据调整)
static const joystick_calib_t calib_left_x = {
    .phys_min = 0,      .phys_center = 1835, .phys_max = 4095,
    .deadzone = 100,    .invert = true
};

static const joystick_calib_t calib_left_y = {
    .phys_min = 0,      .phys_center = 1765, .phys_max = 4095,
    .deadzone = 100,    .invert = true      
};

// 右摇杆校准配置 (根据你的实测数据调整)
static const joystick_calib_t calib_right_x = {
    .phys_min = 0,      .phys_center = 1775, .phys_max = 4095,
    .deadzone = 100,    .invert = false
};

static const joystick_calib_t calib_right_y = {
    .phys_min = 0,      .phys_center = 1830, .phys_max = 4095,
    .deadzone = 100,    .invert = true      
};

// ==================== 核心映射函数 ====================
static uint16_t map_joystick_value(uint16_t raw, const joystick_calib_t* calib)
{
    // HOJA库的默认校准值 (这在hoja_settings.c中已被定义好，对于每个摇杆轴都是一样的)
    const uint16_t LIB_MIN    = 0xFA;    // 250
    const uint16_t LIB_CENTER = 0x740;   // 1856
    const uint16_t LIB_MAX    = 0xF47;   // 3911

    // 限制输入范围
    if (raw < calib->phys_min) raw = calib->phys_min;
    if (raw > calib->phys_max) raw = calib->phys_max;

    // 死区处理
    if (abs(raw - calib->phys_center) <= calib->deadzone) {
        return LIB_CENTER;
    }

    // 计算比例因子
    float ratio;
    if (raw > calib->phys_center) {
        ratio = (float)(raw - calib->phys_center) / (calib->phys_max - calib->phys_center);
    } else {
        ratio = (float)(calib->phys_center - raw) / (calib->phys_center - calib->phys_min);
    }

    // 应用反转逻辑
    if (calib->invert) {
        if (raw > calib->phys_center) {
            return LIB_CENTER - ratio * (LIB_CENTER - LIB_MIN);
        } else {
            return LIB_CENTER + ratio * (LIB_MAX - LIB_CENTER);
        }
    } else {
        if (raw > calib->phys_center) {
            return LIB_CENTER + ratio * (LIB_MAX - LIB_CENTER);
        } else {
            return LIB_CENTER - ratio * (LIB_CENTER - LIB_MIN);
        }
    }
}

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

// 读取单个ADC通道原始值
static int read_adc_raw(joystick_channel_t channel)
{
    int raw = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, joystick_channels[channel], &raw));
    return raw;
}



// We want to do another define for easy setup of our GPIO pins.
// We will use pull-up configuration so we can simply pull each
// pin to ground to count the button as 'pressed'.
#define GPIO_INPUT_PIN_MASK     ( (1ULL << GPIO_BTN_A)|(1ULL << GPIO_BTN_B)|(1ULL << GPIO_BTN_X)|(1ULL << GPIO_BTN_Y)|(1ULL << GPIO_BTN_DPAD_U)|(1ULL << GPIO_BTN_DPAD_L)|(1ULL << GPIO_BTN_DPAD_D)|(1ULL << GPIO_BTN_DPAD_R)|(1ULL << GPIO_BTN_ZL)|(1ULL << GPIO_BTN_ZR)|(1ULL << GPIO_BTN_START)|(1ULL << GPIO_BTN_SELECT))


// We must define our own callback method
// that will be registered with HOJA. When HOJA is initialized
// this function will be repeatedly called, thus scanning the buttons.
// This allows flexibility to scan buttons as you see fit, and you don't
// really have to worry about anything else :)
void local_button_cb()
{
    // First we scan the GPIO. We can do this with register reads though this isn't quite the place to learn how to use this  to its fullest. 
    // Since our highest GPIO used is 31, we do not need to worry about reading the higher GPIO registers. 

    uint32_t register_read_low = REG_READ(GPIO_IN_REG);

    // If we wanted to read GPIO 32 and upwards, we could do this
    // uint32_t register_read_high = REG_READ(GPIO_IN1_REG);

    // From here, we can read out the button data. We have a pointer to the API's internal button structure to set the buttons.

    // The original example code in the HOJA-LIB-ESP32 repo used OR equal, and that proved to be wrong combined with hoja_button_reset()

    // There are some utilities provided by this API that can be useful. In this demo
    // we use the util_getbit demo function which simply returns the bit value from
    // a 32 bit unsigned int. The result is cleaner code that is easier to read :)
    hoja_button_data.dpad_down      = !util_getbit(register_read_low, GPIO_BTN_DPAD_D);
    hoja_button_data.dpad_left      = !util_getbit(register_read_low, GPIO_BTN_DPAD_L);
    hoja_button_data.dpad_right     = !util_getbit(register_read_low, GPIO_BTN_DPAD_R);
    hoja_button_data.dpad_up        = !util_getbit(register_read_low, GPIO_BTN_DPAD_U);

    // ABXY buttons are not labelled as such in the HOJA backend.
    // This is because this core supports many controller types ranging from N64 to XInput.
    // The position of the buttons is more important than the lettering here. 
    hoja_button_data.button_right   = !util_getbit(register_read_low, GPIO_BTN_A);
    hoja_button_data.button_down    = !util_getbit(register_read_low, GPIO_BTN_B);
    hoja_button_data.button_up      = !util_getbit(register_read_low, GPIO_BTN_X);
    hoja_button_data.button_left    = !util_getbit(register_read_low, GPIO_BTN_Y);

    // Intellisense in VS Code with the ESP-IDF extension is very useful if you are
    // unsure of some types and want to quickly get more information on a function
    // or unknown type. Simply right click and select "Go to definition"!
    hoja_button_data.trigger_zl      = !util_getbit(register_read_low, GPIO_BTN_ZL);
    hoja_button_data.trigger_zr      = !util_getbit(register_read_low, GPIO_BTN_ZR);

    hoja_button_data.button_select  = !util_getbit(register_read_low, GPIO_BTN_SELECT);
    hoja_button_data.button_start   = !util_getbit(register_read_low, GPIO_BTN_START);

    // There is a special button which is button_sleep.
    // This is can be set by any button of your choosing as shown.
    // This will send an event to the HOJA event callback system.
    // More documentation on the event callback system will be available
    // at a later time. See the examples folder for the latest updates.

    // The sleep button callback event will execute when you press and hold the select button 
    // for about 3 seconds. Useful for putting the controller to sleep on a battery etc. 
    hoja_button_data.button_sleep   = !util_getbit(register_read_low, GPIO_BTN_SELECT);
}

// This is what the callback function looks like to read the analog stick data.
// This is read once per poll across each controller core. The analog values MUST be
// 12 bit values. If you are getting 8 bit readings, simply bitshift them to the proper resolution.
// 摇杆数据处理函数（带方向反转和死区处理）
void local_analog_cb() 
{
    // 读取原始ADC值
    int ls_x_raw = adc1_get_raw(ADC1_CHANNEL_4); // GPIO32
    int ls_y_raw = adc1_get_raw(ADC1_CHANNEL_5); // GPIO33
    int rs_x_raw = adc1_get_raw(ADC1_CHANNEL_6); // GPIO34
    int rs_y_raw = adc1_get_raw(ADC1_CHANNEL_7); // GPIO35
    
    // 应用校准映射
    hoja_analog_data.ls_x = map_joystick_value(ls_x_raw, &calib_left_x);
    hoja_analog_data.ls_y = map_joystick_value(ls_y_raw, &calib_left_y);
    hoja_analog_data.rs_x = map_joystick_value(rs_x_raw, &calib_right_x);
    hoja_analog_data.rs_y = map_joystick_value(rs_y_raw, &calib_right_y);
}

// The event system callback function is needed, even if you do not use it.
void local_event_cb(hoja_event_type_t type, uint8_t evt, uint8_t param)
{
    const char* TAG = "local_event_cb";

    // This will call when the select button is held.
    if (type == HOJA_EVT_SYSTEM && evt == HEVT_API_SHUTDOWN)
    {
        ESP_LOGI(TAG, "Sleep event triggered!");
    }
}

void app_main(void)
{
    // Set up a tag character array for logging
    const char* TAG = "app_main";

    // 初始化ADC
    adc_init();

    // Set up IO configuration
    gpio_config_t io_conf = {0};

    // Set up IO pins for getting buttons

    // We do not use interrupts here.
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // Apply our previously defined pin mask
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_MASK;
    // Set pins to INPUT mode.
    io_conf.mode = GPIO_MODE_INPUT;
    // Enable internal pull-up resistors on pins.
    // If you have issues, see ESP32 documentation.
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    // Finalize configuration and register it.
    gpio_config(&io_conf);

    // These register functions MUST be called before
    // you try to initialize the API.
    // Pass the functions you've created as parameters.
    hoja_register_button_callback(local_button_cb);
    hoja_register_analog_callback(local_analog_cb);
    hoja_register_event_callback(local_event_cb);

    // We use hoja_err_t for error checking
    // with this API.
    // This helps to differentiate it from other ESP-only functions.
    hoja_err_t err;

    // Attempt initialization.
    err = hoja_init();

    // Check if we initialized Ok
    if (err != HOJA_OK)
    {   
        // If we failed, log this to the console.
        ESP_LOGE(TAG, "Failed to initialize HOJA.");
    }
    else
    {
        // If we got here, HOJA initialized OK.

        // For this example, we will start the XINPUT bluetooth gamepad core.
        // Once the controller turns on, you should see an XINPUT device in your
        // bluetooth pairing menu.

        // For this project, we will start the Nintendo Switch bluetooth gamepad core.
        // Once the controller turns on, it'll automatically try to connect with NS

        hoja_set_core(HOJA_CORE_NS);

        // Attempt to start the core.
        hoja_start_core();
    }
}