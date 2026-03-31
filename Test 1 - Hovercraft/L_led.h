#ifndef L_LED_H
#define L_LED_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
// Limits for allowed yaw range
#define YAW_LIMIT 85.0

// Function to update LED based on yaw
void imu_led_update(float yaw);
void imu_led_init();

#ifdef __cplusplus
}
#endif

#endif
