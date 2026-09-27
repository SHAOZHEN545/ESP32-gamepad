#include "core_switch_imu.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

#define NS_IMU_SENSITIVITY_SIZE 4

typedef struct {
    uint8_t mode;
    uint8_t sensitivity[NS_IMU_SENSITIVITY_SIZE];
    ns_imu_sample_s sample;
} ns_imu_state_s;

static portMUX_TYPE s_imu_lock = portMUX_INITIALIZER_UNLOCKED;
static ns_imu_state_s s_imu_state;

static void pack_i16_le(uint8_t *out, int16_t value)
{
    uint16_t raw = (uint16_t)value;
    out[0] = raw & 0xff;
    out[1] = raw >> 8;
}

static void pack_sample(uint8_t *out, const ns_imu_sample_s *sample)
{
    /* Switch 0x30 wire order is accel Y/X/Z, then gyro Y/X/Z. */
    pack_i16_le(&out[0], sample->ay);
    pack_i16_le(&out[2], sample->ax);
    pack_i16_le(&out[4], sample->az);
    pack_i16_le(&out[6], sample->gy);
    pack_i16_le(&out[8], sample->gx);
    pack_i16_le(&out[10], sample->gz);
}

void ns_imu_init(void)
{
    portENTER_CRITICAL(&s_imu_lock);
    memset(&s_imu_state, 0, sizeof(s_imu_state));
    portEXIT_CRITICAL(&s_imu_lock);
}

void ns_imu_set_mode(uint8_t mode)
{
    portENTER_CRITICAL(&s_imu_lock);
    s_imu_state.mode = mode;
    portEXIT_CRITICAL(&s_imu_lock);
}

bool ns_imu_is_enabled(void)
{
    bool enabled;
    portENTER_CRITICAL(&s_imu_lock);
    enabled = s_imu_state.mode != NS_IMU_MODE_OFF;
    portEXIT_CRITICAL(&s_imu_lock);
    return enabled;
}

void ns_imu_set_sensitivity(const uint8_t *data, uint8_t len)
{
    if (data == NULL) return;
    if (len > NS_IMU_SENSITIVITY_SIZE) len = NS_IMU_SENSITIVITY_SIZE;

    portENTER_CRITICAL(&s_imu_lock);
    memset(s_imu_state.sensitivity, 0, sizeof(s_imu_state.sensitivity));
    memcpy(s_imu_state.sensitivity, data, len);
    portEXIT_CRITICAL(&s_imu_lock);
}

void ns_imu_set_sample(const ns_imu_sample_s *sample)
{
    if (sample == NULL) return;

    portENTER_CRITICAL(&s_imu_lock);
    s_imu_state.sample = *sample;
    portEXIT_CRITICAL(&s_imu_lock);
}

void ns_imu_pack_standard_report(uint8_t *report)
{
    ns_imu_sample_s sample;
    bool enabled;

    if (report == NULL) return;

    portENTER_CRITICAL(&s_imu_lock);
    sample = s_imu_state.sample;
    enabled = s_imu_state.mode != NS_IMU_MODE_OFF;
    portEXIT_CRITICAL(&s_imu_lock);

    memset(&report[NS_IMU_STANDARD_REPORT_OFFSET], 0, NS_IMU_STANDARD_REPORT_SIZE);
    if (!enabled) return;

    for (uint8_t i = 0; i < NS_IMU_SAMPLE_COUNT; ++i) {
        pack_sample(&report[NS_IMU_STANDARD_REPORT_OFFSET + i * NS_IMU_SAMPLE_SIZE], &sample);
    }
}
