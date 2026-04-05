TO DO:
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
