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

/* ===================== USER SETTINGS ===================== */

/* Lift fan command used to keep the craft hovering.
   Tune this experimentally. */
#define LIFT_CRUISE_PERCENT     75

/* Main loop delay.
   Keep this close to the dt assumed inside sensors.c */
#define MAIN_LOOP_DELAY_MS      10

/* Update battery every N loops */
#define BATTERY_LOOP_DIVIDER_MAX 20

int main(void)
{
    uint8_t battery_loop_divider = 0;

    /* ===================== INITIALIZATION ===================== */

    /* Initialize actuator modules */
    timer1_servo_init();
    thrust_init();
    lift_init();

    /* Initialize sensors and battery monitor */
    sensors_init();
    battery_init();

    /* Small startup delay to let hardware settle */
    _delay_ms(300);

    /* First sensor read before defining heading target */
    update_sensors();
    battery_update();

    /* Initial heading target = heading at startup */
    yaw_target = yaw_deg;

    /* Safe startup outputs */
    set_servo_angle(0.0f);
    set_thrust(0);
    set_lift(LIFT_CRUISE_PERCENT);

    /* Optional extra delay before starting mission */
    _delay_ms(300);

    /* ===================== MAIN LOOP ===================== */

    while (1)
    {
        /* Read sensors */
        update_sensors();

        /* Update battery at a slower rate */
        battery_loop_divider++;
        if (battery_loop_divider >= BATTERY_LOOP_DIVIDER_MAX) {
            battery_loop_divider = 0;
            battery_update();
        }

        /* Low battery safety */
        if (battery_is_low()) {
            set_thrust(0);
            set_lift(0);
            set_servo_angle(0.0f);

            while (1) {
                _delay_ms(100);
            }
        }

        /* Update autonomous mission logic */
        update_state();

        /* Apply control outputs for current state */
        apply_control();

        /* Keep loop timing approximately constant */
        _delay_ms(MAIN_LOOP_DELAY_MS);
    }

    return 0;
}
