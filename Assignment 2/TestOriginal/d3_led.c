#define F_CPU 16000000UL

#include <avr/io.h>
#include <stdint.h>
#include "d3_led.h"

// ------------------------------------------------------------
// D3 LED hardware mapping
// D3 is on PB3 / OC2A
// Board LED is active LOW:
//   OCR2A = 255 -> LED OFF
//   OCR2A = 0   -> LED fully ON
// ------------------------------------------------------------

// ------------------------------------------------------------
// Local helper function:
// returns absolute value of a float without needing math.h
// ------------------------------------------------------------
static float d3_abs_float(float x)
{
    if (x < 0.0f)
    {
        return -x;
    }
    return x;
}

// ------------------------------------------------------------
// Converts X-axis acceleration in g into brightness 0..255
//
// Assignment rules:
//   |ax| < 0.08 g  -> 0
//   |ax| > 1.08 g  -> 255
//   linear in between
//
// Formula used in linear region:
//   pwm = ((|ax| - 0.08) / (1.08 - 0.08)) * 255
//
// Since the range is exactly 1.00 g, this simplifies nicely,
// but it is still written in full for clarity.
// ------------------------------------------------------------
uint8_t d3_led_accel_to_pwm(float x_accel_g)
{
    float abs_g;
    float scaled;
    uint16_t pwm;

    abs_g = d3_abs_float(x_accel_g);

    // Below lower threshold -> LED OFF
    if (abs_g < D3_ACCEL_MIN_G)
    {
        return 0;
    }

    // Above upper threshold -> LED fully ON
    if (abs_g > D3_ACCEL_MAX_G)
    {
        return 255;
    }

    // Linear scaling between 0.08 g and 1.08 g
    scaled = (abs_g - D3_ACCEL_MIN_G) / (D3_ACCEL_MAX_G - D3_ACCEL_MIN_G);

    // Convert to 0..255
    pwm = (uint16_t)(scaled * 255.0f);

    // Protection, just in case of rounding effects
    if (pwm > 255)
    {
        pwm = 255;
    }

    return (uint8_t)pwm;
}

// ------------------------------------------------------------
// Initializes Timer2 for Fast PWM on OC2A (PB3)
//
// Timer2 is used here, not Timer1.
// So this does not interfere with your servo code, since your
// servo is already using Timer1 on PB1 / OC1A.
//
// Fast PWM mode:
//   WGM21 = 1, WGM20 = 1
//
// Non-inverting output on OC2A:
//   COM2A1 = 1
//
// Prescaler = 64:
//   PWM frequency ≈ 16MHz / (64 * 256) = 976 Hz
//
// Start with LED OFF:
//   because D3 is active LOW, OCR2A = 255 means OFF
// ------------------------------------------------------------
void d3_led_init(void)
{
    // Set PB3 (OC2A) as output
    DDRB |= (1 << PB3);

    // Timer2 Fast PWM, output on OC2A
    TCCR2A = 0;
    TCCR2B = 0;

    // Fast PWM mode: WGM21 = 1, WGM20 = 1
    TCCR2A |= (1 << WGM21) | (1 << WGM20);

    // Non-inverting PWM on OC2A
    TCCR2A |= (1 << COM2A1);

    // Prescaler = 64
    TCCR2B |= (1 << CS22);

    // Start with LED OFF
    OCR2A = 255;
}

// ------------------------------------------------------------
// Updates the LED brightness based on X-axis acceleration in g
//
// Because D3 is active LOW:
//   brightness 0   -> OCR2A = 255
//   brightness 255 -> OCR2A = 0
// ------------------------------------------------------------
void d3_led_update(float x_accel_g)
{
    uint8_t brightness;

    brightness = d3_led_accel_to_pwm(x_accel_g);

    // Invert because LED is active LOW
    OCR2A = 255 - brightness;
}
