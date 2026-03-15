#ifndef SERVO_CODE_H
#define SERVO_CODE_H

#include <stdint.h>

void timer1_servo_init(void);
void servo_set_from_yaw(int16_t yaw_deg);

#endif
