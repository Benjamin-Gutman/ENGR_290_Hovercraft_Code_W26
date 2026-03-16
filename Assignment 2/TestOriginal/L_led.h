#ifndef IMU_LED_H
#define IMU_LED_H

#include <stdint.h>

// Limits for allowed yaw range
#define YAW_LIMIT 85.0

// Function to update LED based on yaw
void imu_led_update(float yaw);

#endif
