#define F_CPU 16000000UL

#include <avr/io.h>
#include <stdint.h>
#include "thrust.h"

/* ===================== CONFIGURATION ===================== */

/* Thrust fan PWM output pin
   OC2A = PB3 on ATmega328P */
#define THRUST_PWM_DDR    DDRB
#define THRUST_PWM_PORT   PORTB
#define THRUST_PWM_PIN    PB3

/* ===================== STATE ===================== */

static uint8_t thrust_percent = 0;

/* ===================== FUNCTIONS ===================== */

void thrust_init(void)
{
    /* Set PWM pin as output */
    THRUST_PWM_DDR |= (1 << THRUST_PWM_PIN);

    /* Timer2 Fast PWM, non-inverting mode on OC2A */
    TCCR2A = 0;
    TCCR2B = 0;

    TCCR2A |= (1 << WGM21) | (1 << WGM20);   /* Fast PWM */
    TCCR2A |= (1 << COM2A1);                 /* Non-inverting PWM */

    /* Prescaler = 64 */
    TCCR2B |= (1 << CS22);

    OCR2A = 0;
    thrust_percent = 0;
}

void set_thrust(uint8_t percent)
{
    uint16_t duty;

    if (percent > 100) percent = 100;

    thrust_percent = percent;

    /* Map 0..100% to 0..255 */
    duty = ((uint16_t)percent * 255U) / 100U;
    OCR2A = (uint8_t)duty;
}

uint8_t get_thrust(void)
{
    return thrust_percent;
}
