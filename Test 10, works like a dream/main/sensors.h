#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>
#include "IMU.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Global IR distances in centimeters */
extern float left_cm;
extern float front_cm;

/* Raw ADC values for debugging and calibration */
extern uint16_t left_adc;
extern uint16_t front_adc;

/* IMU outputs used by control and state logic */
extern float yaw_deg;
extern float yaw_rate_dps;

/* Full IMU state */
extern imu_state_t imu;

/* Initialize ADC + I2C + IMU */
void sensors_init(void);

/* Update only IR sensors */
void update_ir_sensors(void);

/* Update only IMU using the provided dt */
void update_imu_sensor(float dt_s);

/* Optional wrapper that updates everything */
void update_sensors();

/* Helper to check if an IR reading is inside the useful range */
uint8_t ir_is_valid(float cm);

#ifdef __cplusplus
}
#endif

#endif