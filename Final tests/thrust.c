#define F_CPU 16000000UL

#include <avr/io.h>
#include <stdint.h>
#include "thrust.h"

/* Thrust fan is connected to P3 -> PD5 / OC0B */
#define THRUST_PWM_DDR   DDRD
#define THRUST_PWM_PIN   PD5

static uint8_t thrust_percent = 0;

/* Shared Timer0 setup for both fan channels:
   - OC0A = PD6 = P4 = lift
   - OC0B = PD5 = P3 = thrust */
static void timer0_pwm_init_shared(void)
{
    /* Fast PWM mode, TOP = 0xFF */
    TCCR0A |= (1 << WGM01) | (1 << WGM00);

    /* Prescaler = 64 */
    TCCR0B |= (1 << CS01) | (1 << CS00);
}

void thrust_init(void)
{
    /* Set PD5 as output */
    THRUST_PWM_DDR |= (1 << THRUST_PWM_PIN);

    /* Initialize shared Timer0 mode */
    timer0_pwm_init_shared();

    /* Enable non-inverting PWM on OC0B */
    TCCR0A |= (1 << COM0B1);

    /* Start with thrust off */
    OCR0B = 0;
    thrust_percent = 0;
}

void set_thrust(uint8_t percent)
{
    uint16_t duty;

    if (percent > 100) percent = 100;

    thrust_percent = percent;

    /* Map 0..100% to 0..255 */
    duty = ((uint16_t)percent * 255U) / 100U;
    OCR0B = (uint8_t)duty;
}

uint8_t get_thrust(void)
{
    return thrust_percent;
}