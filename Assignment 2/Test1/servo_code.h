#ifndef SERVO_CODE_H
#define SERVO_CODE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
void timer1_servo_init(void);
void servo_set_from_yaw(int16_t yaw_deg);
#ifdef __cplusplus
}
#endif
#endif
