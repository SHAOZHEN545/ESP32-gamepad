#include "core_switch_imu.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

#define NS_IMU_STANDARD_OFFSET 12
#define NS_IMU_SAMPLE_SIZE 12
#define NS_IMU_SAMPLE_COUNT 3
#define NS_IMU_STANDARD_SIZE (NS_IMU_SAMPLE_SIZE * NS_IMU_SAMPLE_COUNT)
#define NS_IMU_SENSITIVITY_SIZE 4

typedef struct
{
    uint8_t mode;
    uint8_t sensitivity[NS_IMU_SENSITIVITY_SIZE];
    ns_imu_sample_s sample;
} ns_imu_state_s;

static portMUX_TYPE _ns_imu_lock = portMUX_INITIALIZER_UNLOCKED;
static ns_imu_state_s _ns_imu_state = {0};

static void _ns_imu_pack_i16(uint8_t *out, int16_t value)
{
    uint16_t raw = (uint16_t) value;

    out[0] = raw & 0xFF;
    out[1] = (raw >> 8) & 0xFF;
}

static void _ns_imu_pack_sample(uint8_t *out, const ns_imu_sample_s *sample)
{
    _ns_imu_pack_i16(&out[0], sample->ay);
    _ns_imu_pack_i16(&out[2], sample->ax);
    _ns_imu_pack_i16(&out[4], sample->az);
    _ns_imu_pack_i16(&out[6], sample->gy);
    _ns_imu_pack_i16(&out[8], sample->gx);
    _ns_imu_pack_i16(&out[10], sample->gz);
}

void ns_imu_init(void)
{
    portENTER_CRITICAL(&_ns_imu_lock);
    memset(&_ns_imu_state, 0, sizeof(_ns_imu_state));
    portEXIT_CRITICAL(&_ns_imu_lock);
}

void ns_imu_set_mode(uint8_t mode)
{
    portENTER_CRITICAL(&_ns_imu_lock);
    _ns_imu_state.mode = mode;
    portEXIT_CRITICAL(&_ns_imu_lock);
}

uint8_t ns_imu_get_mode(void)
{
    uint8_t mode = 0;

    portENTER_CRITICAL(&_ns_imu_lock);
    mode = _ns_imu_state.mode;
    portEXIT_CRITICAL(&_ns_imu_lock);

    return mode;
}

bool ns_imu_is_enabled(void)
{
    return ns_imu_get_mode() != NS_IMU_MODE_OFF;
}

void ns_imu_set_sensitivity(const uint8_t *data, uint8_t len)
{
    if (data == NULL)
    {
        return;
    }

    if (len > NS_IMU_SENSITIVITY_SIZE)
    {
        len = NS_IMU_SENSITIVITY_SIZE;
    }

    portENTER_CRITICAL(&_ns_imu_lock);
    memset(_ns_imu_state.sensitivity, 0, sizeof(_ns_imu_state.sensitivity));
    memcpy(_ns_imu_state.sensitivity, data, len);
    portEXIT_CRITICAL(&_ns_imu_lock);
}

void ns_imu_set_sample(const ns_imu_sample_s *sample)
{
    if (sample == NULL)
    {
        return;
    }

    portENTER_CRITICAL(&_ns_imu_lock);
    _ns_imu_state.sample = *sample;
    portEXIT_CRITICAL(&_ns_imu_lock);
}

void ns_imu_get_sample(ns_imu_sample_s *out)
{
    if (out == NULL)
    {
        return;
    }

    portENTER_CRITICAL(&_ns_imu_lock);
    *out = _ns_imu_state.sample;
    portEXIT_CRITICAL(&_ns_imu_lock);
}

void ns_imu_pack_standard_report(uint8_t *report)
{
    ns_imu_sample_s sample = {0};

    if (report == NULL)
    {
        return;
    }

    memset(&report[NS_IMU_STANDARD_OFFSET], 0, NS_IMU_STANDARD_SIZE);

    if (!ns_imu_is_enabled())
    {
        return;
    }

    ns_imu_get_sample(&sample);

    for (uint8_t i = 0; i < NS_IMU_SAMPLE_COUNT; i++)
    {
        _ns_imu_pack_sample(&report[NS_IMU_STANDARD_OFFSET + (i * NS_IMU_SAMPLE_SIZE)], &sample);
    }
}
