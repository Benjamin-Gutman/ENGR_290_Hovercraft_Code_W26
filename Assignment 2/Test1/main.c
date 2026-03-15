//Servo attached to P9
//IMU attached
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <math.h>

#include "I2C_driver.c"
#include "IMU_code.c"
#include "servo_code.c"
#include "uart.c"

int main(void)
{
    imu_state_t imu;
    // Structure that stores all IMU data and computed orientation
    //(roll, pitch, yaw)
    //This is created in imu_code
    
    // Initialize communication with IMU
    I2C_init();
    //From I2C_driver
	
    // Initialize MPU6050 sensor
    mpu6050_init();
    //From imu_code

    // Reset IMU internal state
    imu_reset_state(&imu);
    //From imu_code

    // Calibrate IMU 
    imu_calibrate(&imu, 500);
    //From imu_code

    // Initialize servo
    servo_initialize();
    //From servo_code
	
	//Initializes uart funcs
	UartInitialize();
	//Functions you can use;
	// UartPrintString(" ");
	// UartPrint_u16();
	// UartAddNewLine();
	
    while (1)
    {
        // Update IMU estimation
        imu_update(&imu, 0.01f);
		// Read new MPU6050 data and update roll, pitch, and yaw (dt ≈ 0.01 s = 10 ms)
		UartPrint_u16((int16_t)imu.x_m);
        // Send yaw to servo
        servo_set_from_yaw((int16_t)imu.yaw_deg); //Converts float to int

        _delay_ms(10);
    }
}
