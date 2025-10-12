#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "hoja_includes.h"

// ===================================================================================
// --- 按键和HOJA库配置区 ---
// ===================================================================================

#define GPIO_BTN_A          GPIO_NUM_19
#define GPIO_BTN_B          GPIO_NUM_18
#define GPIO_BTN_X          GPIO_NUM_17
#define GPIO_BTN_Y          GPIO_NUM_16
#define GPIO_BTN_DPAD_U     GPIO_NUM_25 
#define GPIO_BTN_DPAD_L     GPIO_NUM_27
#define GPIO_BTN_DPAD_D     GPIO_NUM_26
#define GPIO_BTN_DPAD_R     GPIO_NUM_14
#define GPIO_BTN_ZL         GPIO_NUM_12
#define GPIO_BTN_ZR         GPIO_NUM_4
#define GPIO_BTN_START      GPIO_NUM_1
#define GPIO_BTN_SELECT     GPIO_NUM_5

#define GPIO_INPUT_PIN_MASK ( (1ULL << GPIO_BTN_A)|(1ULL << GPIO_BTN_B)|(1ULL << GPIO_BTN_X)|(1ULL << GPIO_BTN_Y)|(1ULL << GPIO_BTN_DPAD_U)|(1ULL << GPIO_BTN_DPAD_L)|(1ULL << GPIO_BTN_DPAD_D)|(1ULL << GPIO_BTN_DPAD_R)|(1ULL << GPIO_BTN_ZL)|(1ULL << GPIO_BTN_ZR)|(1ULL << GPIO_BTN_START)|(1ULL << GPIO_BTN_SELECT))

// ===================================================================================
// --- 鼠标 (CH9350) 配置区 ---
// ===================================================================================

#define MOUSE_UART_PORT_NUM   UART_NUM_1
#define MOUSE_UART_RX_PIN     13 
#define MOUSE_UART_TX_PIN     23
#define MOUSE_UART_BAUD_RATE  115200
#define MOUSE_BUF_SIZE        (128)

// --- 摇杆转换逻辑配置 ---
#define MOUSE_SENSITIVITY      20.0f // !! 关键参数：鼠标灵敏度，请根据手感调整 !!
#define MOUSE_IDLE_TIMEOUT_US  40000 // 100ms 无操作后摇杆自动回中 (单位: 微秒)

// HOJA库摇杆数值定义
#define RS_CENTER 1984 // 0x7C0
#define RS_MIN    250  // 0xFA
#define RS_MAX    3911 // 0xF47

// 鼠标数据结构体
typedef struct {
    uint8_t buttons;
    int8_t  x;
    int8_t  y;
    int8_t  wheel;
} mouse_data_t;

// 用于任务间通信的队列
static QueueHandle_t mouse_data_queue;

// ===================================================================================
// --- CH9350 鼠标数据读取任务 ---
// ===================================================================================
void ch9350_reader_task(void *pvParameters) {
    uart_config_t uart_config = {
        .baud_rate = MOUSE_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(MOUSE_UART_PORT_NUM, MOUSE_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(MOUSE_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(MOUSE_UART_PORT_NUM, MOUSE_UART_TX_PIN, MOUSE_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    uint8_t *data = (uint8_t *) malloc(MOUSE_BUF_SIZE);
    mouse_data_t mouse_report;

    ESP_LOGI("CH9350", "Starting CH9350 Reader task.");

    while (1) {
        int len = uart_read_bytes(MOUSE_UART_PORT_NUM, data, MOUSE_BUF_SIZE, 20 / portTICK_PERIOD_MS);
        
        if (len > 0) {
            for (int i = 0; i < len; ++i) {
                // 检查帧头和数据标识
                if (data[i] == 0x57 && i + 6 < len && data[i+1] == 0xAB && data[i+2] == 0x02) {
                    mouse_report.buttons = data[i+3];
                    mouse_report.x       = (int8_t)data[i+4];
                    mouse_report.y       = (int8_t)data[i+5];
                    mouse_report.wheel   = (int8_t)data[i+6];

                    // 将解析后的数据发送到队列
                    xQueueSend(mouse_data_queue, &mouse_report, (TickType_t)0);
                    
                    i += 6; // 跳过已处理的帧
                }
            }
        }
    }
    free(data);
}


// ===================================================================================
// --- HOJA 回调函数 ---
// ===================================================================================

// 按键回调 (保持不变)
void local_button_cb()
{
    uint32_t register_read_low = REG_READ(GPIO_IN_REG);
    hoja_button_data.dpad_down      = !util_getbit(register_read_low, GPIO_BTN_DPAD_D);
    hoja_button_data.dpad_left      = !util_getbit(register_read_low, GPIO_BTN_DPAD_L);
    hoja_button_data.dpad_right     = !util_getbit(register_read_low, GPIO_BTN_DPAD_R);
    hoja_button_data.dpad_up        = !util_getbit(register_read_low, GPIO_BTN_DPAD_U);
    hoja_button_data.button_right   = !util_getbit(register_read_low, GPIO_BTN_A);
    hoja_button_data.button_down    = !util_getbit(register_read_low, GPIO_BTN_B);
    hoja_button_data.button_up      = !util_getbit(register_read_low, GPIO_BTN_X);
    hoja_button_data.button_left    = !util_getbit(register_read_low, GPIO_BTN_Y);
    hoja_button_data.trigger_zl     = !util_getbit(register_read_low, GPIO_BTN_ZL);
    hoja_button_data.trigger_zr     = !util_getbit(register_read_low, GPIO_BTN_ZR);
    hoja_button_data.button_select  = !util_getbit(register_read_low, GPIO_BTN_SELECT);
    hoja_button_data.button_start   = !util_getbit(register_read_low, GPIO_BTN_START);
    hoja_button_data.button_sleep   = !util_getbit(register_read_low, GPIO_BTN_SELECT);
}

// 辅助函数：用于限制摇杆值范围
static int32_t clamp_stick_value(int32_t val)
{
    if (val < RS_MIN) return RS_MIN;
    if (val > RS_MAX) return RS_MAX;
    return val;
}

// 模拟摇杆回调 (核心修改区域)
void local_analog_cb()
{
    // // 静态变量，用于保存摇杆的当前状态
    // static int32_t current_rs_x = RS_CENTER;
    // static int32_t current_rs_y = RS_CENTER;
    // static int64_t last_mouse_move_time_us = 0;

    // mouse_data_t received_mouse_data;

    // // 检查队列中是否有新的鼠标数据
    // if (xQueueReceive(mouse_data_queue, &received_mouse_data, (TickType_t)0) == pdPASS)
    // {
    //     // 如果有鼠标移动
    //     if (received_mouse_data.x != 0 || received_mouse_data.y != 0)
    //     {
    //         last_mouse_move_time_us = esp_timer_get_time(); // 更新最后移动时间

    //         // 计算新的摇杆位置
    //         current_rs_x += (int32_t)(received_mouse_data.x * MOUSE_SENSITIVITY);
    //         current_rs_y += (int32_t)(received_mouse_data.y * MOUSE_SENSITIVITY);

    //         // 限制摇杆范围
    //         current_rs_x = clamp_stick_value(current_rs_x);
    //         current_rs_y = clamp_stick_value(current_rs_y);
    //     }
    // }

    // // 检查自动回中超时
    // if (esp_timer_get_time() - last_mouse_move_time_us > MOUSE_IDLE_TIMEOUT_US)
    // {
    //     // 如果超时，则将摇杆置于中心
    //     current_rs_x = RS_CENTER;
    //     current_rs_y = RS_CENTER;
    // }
    
    // // ---- 更新HOJA摇杆数据 ----
    // // 左摇杆保持中心
    // hoja_analog_data.ls_x = 0x740;
    // hoja_analog_data.ls_y = 0x740;
    
    // // 右摇杆使用我们计算出的值
    // hoja_analog_data.rs_x = current_rs_x;
    // hoja_analog_data.rs_y = current_rs_y;
}

// 事件回调 (保持不变)
void local_event_cb(hoja_event_type_t type, uint8_t evt, uint8_t param)
{
    const char* TAG = "local_event_cb";
    if (type == HOJA_EVT_SYSTEM && evt == HEVT_API_SHUTDOWN)
    {
        ESP_LOGI(TAG, "Sleep event triggered!");
    }
}


// ===================================================================================
// --- 主函数 app_main ---
// ===================================================================================
void app_main(void)
{
    const char* TAG = "app_main";

    // --- 初始化GPIO按键 ---
    gpio_config_t io_conf = {0};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_MASK;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
    
    // --- 创建鼠标数据队列 ---
    mouse_data_queue = xQueueCreate(10, sizeof(mouse_data_t)); // 队列长度10，足以缓冲

    // --- 创建并启动鼠标读取任务 ---
    xTaskCreate(ch9350_reader_task, "ch9350_reader_task", 4096, NULL, 10, NULL);

    ESP_LOGI(TAG, "CH9350 Task and Queue created.");

    // --- 注册HOJA回调函数 ---
    hoja_register_button_callback(local_button_cb);
    hoja_register_analog_callback(local_analog_cb);
    hoja_register_event_callback(local_event_cb);

    // --- 初始化并启动HOJA ---
    hoja_err_t err = hoja_init();
    if (err != HOJA_OK)
    { 
        ESP_LOGE(TAG, "Failed to initialize HOJA.");
    }
    else
    {
        ESP_LOGI(TAG, "HOJA Initialized OK.");
        hoja_set_core(HOJA_CORE_NS); // 设置为 NS (Nintendo Switch) 核心
        hoja_start_core();
        ESP_LOGI(TAG, "HOJA Core Started.");
    }
}