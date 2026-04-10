TO DO:
1. Compile and linking
/* Compile everything and make sure the full project builds correctly.
   Verify that all files are included, all functions are defined,
   and there are no syntax, linking, or duplicate symbol errors. */

2. TURN state exit condition
/* Verify that the turn exit condition is correct.
   The hovercraft should leave STATE_TURN only when the current yaw
   is close enough to yaw_target. */

3. Sensor and servo sign conventions
/* Verify all sign conventions.
   Confirm that yaw increases/decreases in the expected direction,
   that positive servo angles turn the craft the intended way,
   and that sensor thresholds correspond to real track conditions. */

4. Real pin mapping
/* Verify that all software pin definitions match the real hardware wiring.
   Sensor ADC channels, servo output, lift PWM, thrust PWM,
   and battery monitor input must all be connected to the correct pins. */

5. Fan driver compatibility
/* Verify that the fan control method matches the real hardware.
   Direct PWM works for transistor/MOSFET-driven fans,
   but ESC-driven motors require a different control signal. */

6. Real sensor behavior
/* Verify real sensor behavior.
   Check that left_cm and front_cm respond correctly to wall distance,
   remain reasonably stable, and reflect the true geometry of the track. */

7. IMU yaw drift
/* Verify IMU yaw drift.
   After calibration, yaw_deg should remain reasonably stable when the craft is stationary.
   Excessive drift will make heading control unreliable. */

8. Thrust values per state
/* Verify thrust values for each state.
   Straight motion, turn preparation, turning, and recentering
   should each use thrust levels that match the intended behavior. */

VERIFY PINS:
left IR → ADC0 / PC0
front IR → ADC1 / PC1
battery → ADC2 / PC2
lift PWM → PD6 / OC0A
thrust PWM → PB3 / OC2A
servo → PB1 / OC1A





///DONE
1 - ADD FAN CODE (Lift and thrust)
2 - ADD SENSOR CODES
3 - ADD BATTERY CODE
4 - CREATE MAIN


main should look something like this:

...
int main(void){
  
  init_all();//All initializers

  _delay_ms(100); //Needs a small delay

  yaw_target = yaw_deg; //First target yaw is the direction the craft is facing when turned on
  
  while (1)
  {
    update_sensors(); //From sensors.h
    //Just reads values of yaw from IMU and sensor values and assigns them to global variables:
    //left_cm 
    //right_cm
    //yaw_deg
    
      update_state();
      apply_control();
      _delay_ms(10);
  }

}


an example of sensors.h:

#include "sensors.h"
#include "IR_CODE(S)"

float left_cm;
float front_cm;
float yaw_deg;

void update_sensors(void)
{
    // Read IR sensors
    left_cm  = read_left_ir();
    front_cm = read_front_ir();

    // Update IMU
    imu_update(&imu, dt);
    yaw_deg = imu.yaw;
}
