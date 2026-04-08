#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>
#include "IMU.h"

#ifdef __cplusplus
extern "C" {
#endif

extern float left_cm;
extern float front_cm;

extern uint16_t left_adc;
extern uint16_t front_adc;

extern float yaw_deg;
extern float yaw_rate_dps;

extern imu_state_t imu;

void sensors_init(void);
void update_sensors(void);
uint8_t ir_is_valid(float cm);

#ifdef __cplusplus
}
#endif

#endif
