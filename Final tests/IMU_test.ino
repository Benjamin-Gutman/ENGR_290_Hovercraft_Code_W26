#define F_CPU 16000000UL

#include <Arduino.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

#include "servo.h"
#include "thrust.h"
#include "lift.h"
#include "sensors.h"
#include "battery.h"
#include "state_machine.h"
#include "control.h"

void setup()
{
    Serial.begin(115200);

    timer1_servo_init();
    thrust_init();
    lift_init();

    sensors_init();
    battery_init();

    delay(300);

    update_sensors();
    battery_update();

    yaw_target = yaw_deg;

    set_servo_angle(0.0f);
    set_thrust(0);
    set_lift(100);

    delay(300);
}

void loop()
{
    update_sensors();

    Serial.print("YAW: ");
    Serial.print(imu.yaw_deg, 3);
    Serial.println(" deg");

    delay(10);
}