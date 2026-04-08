#ifndef THRUST_H
#define THRUST_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Initialize thrust fan PWM output */
void thrust_init(void);

/* Set thrust fan command in percent: 0..100 */
void set_thrust(uint8_t percent);

/* Return last commanded thrust percentage */
uint8_t get_thrust(void);

#ifdef __cplusplus
}
#endif

#endif