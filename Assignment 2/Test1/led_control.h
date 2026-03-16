#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void led_init(void);
void led_set_axg(float ax_g);

#ifdef __cplusplus
}
#endif

#endif