#include <stdio.h>
#include "hoja_includes.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"

// We must define our GPIO. Doing our own
// defines in this way makes the code easier to read and change later on.
#define GPIO_BTN_A          GPIO_NUM_19
#define GPIO_BTN_B          GPIO_NUM_18
#define GPIO_BTN_X          GPIO_NUM_22
#define GPIO_BTN_Y          GPIO_NUM_21
#define GPIO_BTN_DPAD_U     GPIO_NUM_25 
#define GPIO_BTN_DPAD_L     GPIO_NUM_27
#define GPIO_BTN_DPAD_D     GPIO_NUM_26
#define GPIO_BTN_DPAD_R     GPIO_NUM_14
#define GPIO_BTN_ZL         GPIO_NUM_13
#define GPIO_BTN_ZR         GPIO_NUM_23
#define GPIO_BTN_START      GPIO_NUM_12
#define GPIO_BTN_SELECT     GPIO_NUM_4

// We want to do another define for easy setup of our GPIO pins.
// We will use pull-up configuration so we can simply pull each
// pin to ground to count the button as 'pressed'.
#define GPIO_INPUT_PIN_MASK     ( (1ULL << GPIO_BTN_A)|(1ULL << GPIO_BTN_B)|(1ULL << GPIO_BTN_X)|(1ULL << GPIO_BTN_Y)|(1ULL << GPIO_BTN_DPAD_U)|(1ULL << GPIO_BTN_DPAD_L)|(1ULL << GPIO_BTN_DPAD_D)|(1ULL << GPIO_BTN_DPAD_R)|(1ULL << GPIO_BTN_ZL)|(1ULL << GPIO_BTN_ZR)|(1ULL << GPIO_BTN_START)|(1ULL << GPIO_BTN_SELECT))

// --- CH9350 Mouse UART Configuration ---
#define UART_PORT_NUM       UART_NUM_2
#define UART_RX_PIN         16 // CH9350L TXD Connects to this pin
#define UART_TX_PIN         17 // Required for setup, but not used for sending
#define UART_BAUD_RATE      115200
#define BUF_SIZE            (1024)

static const char *TAG = "CH9350_PARSER";

// --- Global variables for Mouse Data ---
// These are volatile because they are accessed by two different tasks.
// mouse_delta accumulates movement data from the UART task.
volatile int16_t mouse_delta_x = 0;
volatile int16_t mouse_delta_y = 0;

// --- Right Stick Constants for HOJA (12-bit values) ---
#define JOYSTICK_RS_CENTER  (0x740 + 0x80) // 1984
#define JOYSTICK_RS_MIN     0xFA           // 250
#define JOYSTICK_RS_MAX     0xF47          // 3911

// --- Mouse to Stick Conversion Settings ---
#define MOUSE_SENSITIVITY       250.0f // ★★★ 您可以调整这个值来改变视角移动速度 ★★★
#define MOUSE_IDLE_TIMEOUT_MS   40   // 鼠标停止移动后，摇杆自动回中前的延迟（毫秒）

static uint32_t last_mouse_move_time_ms = 0; // 追踪上次鼠标移动的时间

// Helper function to keep stick values within the valid 12-bit range
static inline uint16_t clamp_stick_value(int val)
{
    if (val < JOYSTICK_RS_MIN) return JOYSTICK_RS_MIN;
    if (val > JOYSTICK_RS_MAX) return JOYSTICK_RS_MAX;
    return (uint16_t)val;
}


void uart_init(void) {
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "UART driver installed and configured for RAW data reading.");
}

void ch9350_reader_task(void *pvParameters) {
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    
    if (data == NULL) {
        ESP_LOGE(TAG, "Failed to malloc data buffer! Task aborting.");
        vTaskDelete(NULL); 
    }

    ESP_LOGI(TAG, "Starting CH9350 PARSER task.");

    while (1) {
        int len = uart_read_bytes(UART_PORT_NUM, data, BUF_SIZE, 0); 
        
        if (len > 0) {
            for (int i = 0; i < len; ++i) {
        
                // 改为 vTaskDelay(1)
                if ((i > 0) && (i % 256) == 0) {
                    vTaskDelay(1);
                }

                if (data[i] == 0x57 && i + 6 < len && data[i+1] == 0xAB && data[i+2] == 0x02) {
                    mouse_delta_x += (int8_t)data[i+4];
                    mouse_delta_y += (int8_t)data[i+5];
                    i += 6;
                }
            }
        }
        
        // 改为 vTaskDelay(1)
        vTaskDelay(1); 
    }
    free(data);
}



// We must define our own callback method
// that will be registered with HOJA. When HOJA is initialized
// this function will be repeatedly called, thus scanning the buttons.
// This allows flexibility to scan buttons as you see fit, and you don't
// really have to worry about anything else :)
void local_button_cb()
{
    // First we scan the GPIO. We can do this with register reads
    // though this isn't quite the place to learn how to use this
    // to its fullest. Since our highest GPIO used is 31, we do not
    // need to worry about reading the higher GPIO registers. 

    uint32_t register_read_low = REG_READ(GPIO_IN_REG);

    // If we wanted to read GPIO 32 and upwards,
    // we could do this
    // uint32_t register_read_high = REG_READ(GPIO_IN1_REG);

    // From here, we can read out the button data.
    // We have a pointer to the API's internal button
    // structure to set the buttons.

    // Please note that it's good practice here to use OR equals when setting the buttons.
    // This will allow the buttons to be scanned many times in between controller updates to
    // the host/console device. The buttons states are automatically reset by the API.

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
void local_analog_cb()
{
    // Keep Left Stick centered for now
    hoja_analog_data.ls_x = 0x740;
    hoja_analog_data.ls_y = 0x740;

    // --- Right Stick controlled by Mouse ---

    // Atomically read and reset the global mouse deltas
    int current_delta_x = mouse_delta_x;
    mouse_delta_x = 0;
    int current_delta_y = mouse_delta_y;
    mouse_delta_y = 0;

    if (current_delta_x != 0 || current_delta_y != 0)
    {
        // Mouse has moved, update the timestamp
        last_mouse_move_time_ms = esp_log_timestamp();

        // Calculate new stick positions
        int stick_x = JOYSTICK_RS_CENTER + (int)(current_delta_x * MOUSE_SENSITIVITY);
        
        // Invert Y-axis for typical FPS camera control (move mouse up -> look up)
        int stick_y = JOYSTICK_RS_CENTER - (int)(current_delta_y * MOUSE_SENSITIVITY);

        // Assign the clamped values to the right stick
        hoja_analog_data.rs_x = clamp_stick_value(stick_x);
        hoja_analog_data.rs_y = clamp_stick_value(stick_y);
    }
    else
    {
        // Mouse has not moved, check for idle timeout to re-center the stick
        if ((esp_log_timestamp() - last_mouse_move_time_ms) > MOUSE_IDLE_TIMEOUT_MS)
        {
            hoja_analog_data.rs_x = JOYSTICK_RS_CENTER;
            hoja_analog_data.rs_y = JOYSTICK_RS_CENTER;
        }
    }
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

    // Initialize UART and start the CH9350 reader task
    uart_init();
    xTaskCreate(ch9350_reader_task, "ch9350_reader_task", 4096, NULL, 5, NULL);

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
        hoja_set_core(HOJA_CORE_NS);

        // Attempt to start the core.
        hoja_start_core();
    }
}