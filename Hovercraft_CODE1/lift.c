#define F_CPU 16000000UL

#include <avr/io.h>
#include <stdint.h>
#include "lift.h"

/* ===================== CONFIGURATION ===================== */

/* Lift fan PWM output pin
   OC0A = PD6 on ATmega328P */
#define LIFT_PWM_DDR    DDRD
#define LIFT_PWM_PORT   PORTD
#define LIFT_PWM_PIN    PD6

/* ===================== STATE ===================== */

static uint8_t lift_percent = 0;

/* ===================== FUNCTIONS ===================== */

void lift_init(void)
{
    /* Set PWM pin as output */
    LIFT_PWM_DDR |= (1 << LIFT_PWM_PIN);

    /* Timer0 Fast PWM, non-inverting mode on OC0A */
    TCCR0A = 0;
    TCCR0B = 0;

    TCCR0A |= (1 << WGM01) | (1 << WGM00);   /* Fast PWM */
    TCCR0A |= (1 << COM0A1);                 /* Non-inverting PWM */

    /* Prescaler = 64 */
    TCCR0B |= (1 << CS01) | (1 << CS00);

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
