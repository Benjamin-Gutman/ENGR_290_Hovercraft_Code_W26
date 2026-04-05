#ifndef LIFT_H
#define LIFT_H

#include <stdint.h>

/* Initialize lift fan PWM output */
void lift_init(void);

/* Set lift fan command in percent: 0..100 */
void set_lift(uint8_t percent);

/* Return last commanded lift percentage */
uint8_t get_lift(void);

#endif
