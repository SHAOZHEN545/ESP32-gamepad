#ifndef CORE_SWITCH_IMU_H
#define CORE_SWITCH_IMU_H

#include <stdbool.h>
#include <stdint.h>

/* Nintendo's standard 0x30 report stores three 12-byte IMU samples after
 * the first 12 bytes of controller state. */
#define NS_IMU_STANDARD_REPORT_OFFSET 12
#define NS_IMU_SAMPLE_SIZE 12
#define NS_IMU_SAMPLE_COUNT 3
#define NS_IMU_STANDARD_REPORT_SIZE (NS_IMU_SAMPLE_SIZE * NS_IMU_SAMPLE_COUNT)

typedef enum {
    NS_IMU_MODE_OFF = 0x00,
    NS_IMU_MODE_RAW = 0x01,
    NS_IMU_MODE_QUATERNION = 0x02,
} ns_imu_mode_t;

typedef struct {
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
} ns_imu_sample_s;

void ns_imu_init(void);
void ns_imu_set_mode(uint8_t mode);
bool ns_imu_is_enabled(void);
void ns_imu_set_sensitivity(const uint8_t *data, uint8_t len);
void ns_imu_set_sample(const ns_imu_sample_s *sample);
void ns_imu_pack_standard_report(uint8_t *report);

#endif
