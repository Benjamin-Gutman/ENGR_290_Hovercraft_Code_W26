#define F_CPU 16000000UL

#include <util/delay.h>
#include <stdint.h>

#include "thrust.h"
#include "lift.h"

/*
 * Fan test program
 *
 * This test does the following:
 * 1. Initializes lift and thrust PWM outputs
 * 2. Tests lift fan alone
 * 3. Tests thrust fan alone
 * 4. Tests both fans together
 *
 * This code assumes:
 * - lift fan is controlled by set_lift(percent)
 * - thrust fan is controlled by set_thrust(percent)
 * - both accept PWM percentages from 0 to 100
 */





static void test_both_together(void)
{
    /* Combined test: lift constant, thrust changing */
     set_lift(0);
    set_lift(100);
    _delay_ms(1000);

    set_thrust(50);
    _delay_ms(1000);

    set_thrust(100);
    _delay_ms(1000);

    set_thrust(50);
    _delay_ms(1000);

    set_thrust(100);
    _delay_ms(1000);

    set_lift(0);
    _delay_ms(1000);
}

int main(void)
{
    /* Initialize fan control modules */
    lift_init();
    thrust_init();


    while (1)
    {
        

    set_thrust(100);
    set_lift(100);

        /* Pause before repeating */
    
    }

    return 0;
}