#ifndef D3_LED_H
#define D3_LED_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ------------------------------------------------------------
// D3 LED control using Timer2 PWM on PB3 (OC2A)
//
// Behaviour required by the assignment:
// - OFF when |ax| < 0.08 g
// - 100% ON when |ax| > 1.08 g
// - Linear brightness in between
//
// Input to update function:
// - x_accel_g = X-axis acceleration in units of g
// ------------------------------------------------------------

// Lower threshold in g: below this, LED is OFF
#define D3_ACCEL_MIN_G  0.08f

// Upper threshold in g: above this, LED is fully ON
#define D3_ACCEL_MAX_G  1.08f

// Initializes PB3 / OC2A and Timer2 for PWM control of D3
void d3_led_init(void);

// Updates D3 brightness from X-axis acceleration in g
void d3_led_update(float x_accel_g);

// Converts |x_accel_g| into a brightness value from 0 to 255
uint8_t d3_led_accel_to_pwm(float x_accel_g);

#ifdef __cplusplus
}
#endif

#endif
