//
// Created by Sarkis Seraydarian on 2026-04-03.
//

#ifndef SERVO_H
#define SERVO_H
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
//Initialize function - must be included in main - NOT in loop
void timer1_servo_init(void);

//Set Servo function, used in control
void set_servo_angle(float angle_deg);
#ifdef __cplusplus
}
#endif


#endif //SERVO_H
