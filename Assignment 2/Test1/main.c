#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <math.h>

#include "I2C_driver.h"
#include "IMU_code.h"
#include "servo_code.h"
#include "uart.h"
#include "led_control.h"

int main(void)
{
 
    uint16_t print_count = 0;
    int32_t x_cm;
    int16_t yaw_deg_int;
    int16_t vel;
    int16_t acc;

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
    timer1_servo_init();
    //From servo_code
    UartInitialize();
	
	// Initiallize LED PWM accel control
	led_init();

	

    while (1)
    {
        // Update IMU estimation
        imu_update(&imu, 0.01f);
	// Read new MPU6050 data and update roll, pitch, and yaw (dt ≈ 0.01 s = 10 ms)
		// Send accel_x_g to LED with PWM brightness control
		led_set_axg(imu.ax_g);
        // Send yaw to servo
        servo_set_from_yaw((int16_t)imu.yaw_deg); //Converts float to int
// Print about once per second instead of every 10 ms
        print_count++;
        if (print_count >= 100) {
            print_count = 0;

            x_cm = (int32_t)(imu.x_m * 100.0f);
            yaw_deg_int = (int16_t)(imu.yaw_deg);
            vel = (int16_t)(imu.vx_mps * 100.0f);
            acc = (int16_t)(imu.ax_linear_mps2 * 100.0f);

            UartPrintString("X = ");
           UartPrint_float(x_cm);
            UartPrintString(" cm   Yaw = ");
            UartPrint_float(yaw_deg_int);
            UartPrintString(" deg  Velocity = ");
             UartPrint_float(vel);
            UartPrintString(" cm/s  Acceleration = ");
            UartPrint_float(acc);

            UartAddNewLine();
        _delay_ms(10);
    }
    }}
