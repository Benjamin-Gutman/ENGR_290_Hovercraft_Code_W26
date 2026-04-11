//IMU works
//sensor works

#define F_CPU 16000000UL

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

/* Control loop period = 10 ms */
#define LOOP_DT_S 0.020f

int main(void)
{
    uint8_t ir_divider = 0;
    uint8_t battery_divider = 0;

    // ---- INIT ----
    timer1_servo_init();
    thrust_init();
    lift_init();

    sensors_init();
    battery_init();

    _delay_ms(400);

    // Initial read
    update_sensors();
    battery_update();
  

    update_imu_sensor(LOOP_DT_S);
     yaw_target = yaw_deg;

    set_servo_angle(0.0f);
    set_thrust(0);
    set_lift(100);   // stronger lift for stability

    _delay_ms(300);

    static uint8_t ir_div = 0; //COUNTING sensor udpates

    // ---- MAIN LOOP ----
    while (1)
    {
      

        //SENSORS UPDATE ONCE EVERY 50ms instead of every 10ms
        if (++ir_div >= 10){
            ir_div = 0;
            update_sensors();
        }
        
        update_imu_sensor(LOOP_DT_S);
          //updates IMU according to loop 

        // ---- SLOW IR UPDATE (every 5 loops ≈ 50 ms) ----
        
        // ---- BATTERY UPDATE (slow) ----
        battery_divider++;
        if (battery_divider >= 50) {   // every ~0.5 sec
            battery_divider = 0;
            battery_update();
        }

        // if (battery_is_low()) {
        //     set_thrust(0);
        //     set_lift(0);
        //     set_servo_angle(0.0f);
        // }


        // ---- CONTROL ----
       update_state();
       apply_control();

    
    //NOTE TO SARKIS

   Serial.print(" left ");
 Serial.print(front_cm);
 Serial.println("  deg ");

//This is good, test IMU AGAIN, for final, either keep serial.print();
    _delay_ms(10);  // EXACTLY matches LOOP_DT_S
    }

    return 0;
}
