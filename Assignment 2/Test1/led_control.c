#include <avr/io.h>
#include <stdint.h>
#include "led_control.h"

#define D3_ACCEL_OFF_G   0.08f
#define D3_ACCEL_FULL_G  1.08f

//Pin assignment: LED D3 (PWM) - PD3 (OC2B) - Timer2 Channel B
#define D3_PWM_DDR   DDRD
#define D3_PWM_PORT  PORTD
#define D3_PWM_BIT   PD3

void led_init(void)
{
    D3_PWM_DDR |= (1 << D3_PWM_BIT);

    TCCR2A = (1 << WGM21) | (1 << WGM20) | (1 << COM2B1);
    TCCR2B = (1 << CS22);

    OCR2B = 0;
}

static uint8_t axg2pwm(float ax_g)
{
    float a;
    float scaled;

    if (ax_g < 0.0f)
        a = -ax_g;
    else
        a = ax_g;

    if (a <= D3_ACCEL_OFF_G)
        return 0;

    if (a >= D3_ACCEL_FULL_G)
        return 255;

    scaled = (a - D3_ACCEL_OFF_G) * 255.0f /
             (D3_ACCEL_FULL_G - D3_ACCEL_OFF_G);

    return (uint8_t)(scaled + 0.5f);
}

void led_set_axg(float ax_g)
{
    OCR2B = axg2pwm(ax_g);
}