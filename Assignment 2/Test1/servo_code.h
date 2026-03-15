#ifndef SERVO_CODE_H
#define SERVO_CODE_H

#include <stdint.h>

void servo_initialize(void);
void servo_set_from_yaw(int16_t yaw_deg);

#endif
