#define F_CPU 16000000UL

#include <util/delay.h>
#include <stdint.h>

#include "servo.h"
#include "thrust.h"
#include "lift.h"
#include "sensors.h"
#include "battery.h"
#include "state_machine.h"
#include "control.h"

/* Lift fan command used to keep the craft hovering.
   This value must be tuned experimentally. */
#define LIFT_CRUISE_PERCENT   75

int main(void)
{
    /* This divider is used so the battery is not updated every single loop */
    uint8_t battery_loop_divider = 0;

    /* ===================== INITIALIZATION ===================== */

    /* Initialize actuator modules first */
    timer1_servo_init();
    thrust_init();
    lift_init();

    /* Initialize sensors */
    sensors_init();
    battery_init();

    /* Small startup delay to let hardware settle */
    _delay_ms(300);

    /* First sensor update before defining the initial yaw target */
    update_sensors();
    battery_update();

    /* The first heading target is the direction the craft is facing at startup */
    yaw_target = yaw_deg;

    /* Start from safe outputs */
    set_servo_angle(0.0f);
    set_thrust(0);

    /* Start lift fan so the craft can hover */
    set_lift(LIFT_CRUISE_PERCENT);

    /* Optional extra delay before entering the main loop */
    _delay_ms(300);

    /* ===================== MAIN LOOP ===================== */

    while (1)
    {
        /* Read IR sensors and IMU */
        update_sensors();

        /* Update battery less frequently than the main control loop */
        battery_loop_divider++;
        if (battery_loop_divider >= 20) {
            battery_loop_divider = 0;
            battery_update();
        }

        /* Optional low-battery safety behavior */
        if (battery_is_low()) {
            set_thrust(0);
            set_lift(0);
            set_servo_angle(0.0f);

            /* Stay here if battery is too low */
            while (1) {
                _delay_ms(100);
            }
        }

        /* Update autonomous logic */
        update_state();

        /* Apply servo + thrust control according to the current state */
        apply_control();

        /* Main control loop period: about 10 ms */
        _delay_ms(10);
    }

    return 0;
}
