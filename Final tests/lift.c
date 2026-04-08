#include <avr/io.h>
#include <stdint.h>
#include "lift.h"

/* Lift fan is connected to P4 -> PD6 / OC0A */
#define LIFT_PWM_DDR    DDRD
#define LIFT_PWM_PIN    PD6

static uint8_t lift_percent = 0;

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

void lift_init(void)
{
    /* Set PD6 as output */
    LIFT_PWM_DDR |= (1 << LIFT_PWM_PIN);

    /* Initialize shared Timer0 mode */
    timer0_pwm_init_shared();

    /* Enable non-inverting PWM on OC0A */
    TCCR0A |= (1 << COM0A1);

    /* Start with lift off */
    OCR0A = 0;
    lift_percent = 0;
}

void set_lift(uint8_t percent)
{
    uint16_t duty;

    if (percent > 100) percent = 100;

    lift_percent = percent;

    /* Map 0..100% to 0..255 */
    duty = ((uint16_t)percent * 255U) / 100U;
    OCR0A = (uint8_t)duty;
}

uint8_t get_lift(void)
{
    return lift_percent;
}