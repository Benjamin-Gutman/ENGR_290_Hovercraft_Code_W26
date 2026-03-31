#ifndef FAN_H
#define FAN_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ------------------------------------------------------------

// ------------------------------------------------------------

#define thrust PD6
#define lift PD4

void fan_initialization(void);
void set_max_speed(void);
void set_half_speed(void);
void set_speed(uint8_t pwm);
void set_off(void);

#ifdef __cplusplus
}
#endif

#endif
