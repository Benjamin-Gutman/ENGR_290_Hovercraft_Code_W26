#include <avr/io.h>
#include "L_led.h"

// LED L pin definition
// On this board LED L is connected to PB5
#define LED_L_PORT PORTB
#define LED_L_DDR  DDRB
#define LED_L_PIN  PB5

//--------------------------------------------------
// Initialize LED (call once in main)
//--------------------------------------------------
void imu_led_init(void)
{
    LED_L_DDR |= (1 << LED_L_PIN);   // set as output
    LED_L_PORT |= (1 << LED_L_PIN);  // LED OFF (active LOW)
}

//--------------------------------------------------
// Turn LED ON
//--------------------------------------------------
static inline void led_on(void)
{
    LED_L_PORT &= ~(1 << LED_L_PIN);   // active LOW
}

//--------------------------------------------------
// Turn LED OFF
//--------------------------------------------------
static inline void led_off(void)
{
    LED_L_PORT |= (1 << LED_L_PIN);
}

//--------------------------------------------------
// Check yaw and control LED
//--------------------------------------------------
void imu_led_update(float yaw)
{
    if ((yaw > YAW_LIMIT) || (yaw < -YAW_LIMIT))
    {
        led_off();   // yaw outside ±85°
    }
    else
    {
        led_on();  // yaw within range
    }
}
