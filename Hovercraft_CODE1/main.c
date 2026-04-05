 sensors_init();
    battery_init();

    _delay_ms(300);

    /* First sensor update before setting the initial heading target */
    update_sensors();
    battery_update();

    yaw_target = yaw_deg;

    /* Start lift fan */
    set_lift(LIFT_CRUISE_PERCENT);

    while (1)
    {
        update_sensors();

        /* Battery does not need to be updated every loop */
        battery_loop_divider++;
        if (battery_loop_divider >= 20) {
            battery_loop_divider = 0;
            battery_update();
        }

        /* Optional simple low-battery failsafe */
        if (battery_is_low()) {
            set_thrust(0);
            set_lift(0);
            set_servo_angle(0.0f);

            while (1) {
                _delay_ms(100);
            }
        }

        update_state();
        apply_control();

        _delay_ms(10);
    }

    return 0;
}
