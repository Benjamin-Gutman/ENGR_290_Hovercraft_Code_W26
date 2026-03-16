//Servo attached to P9
//IMU attached to P7 or P19
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <math.h>

#include "I2C_driver.h"
#include "IMU_code.h"
#include "servo_code.h"
#include "uart.h"
#include "d3_led.h"
#include "L_led.h"


int main(void)
{
    imu_state_t imu;
    uint16_t print_count = 0;
    int16_t x_cm;
    int16_t yaw_deg_int;
    int16_t vel;
    int16_t acc;

    // Initialize communication with IMU
    I2C_init();

    // Initialize MPU6050 sensor
    mpu6050_init();

    // Apply IMU configuration before calibration
    mpu6050_set_accel_range(AFS_2G);
    mpu6050_set_gyro_range(GFS_250DPS);
    mpu6050_set_dlpf(3);
    mpu6050_set_sample_rate_div(9);

    // Reset IMU internal state
    imu_reset_state(&imu);

    // Calibrate IMU while completely still
    imu_calibrate(&imu, 500);

    // Initialize servo
    timer1_servo_init();

    // Initialize UART
    UartInitialize();

    //Initialize L Led
    imu_led_init();

    d3_led_init();

    while (1)
    {
        // Update IMU estimation
        imu_update(&imu, 0.01f);

        // Send yaw to servo
        servo_set_from_yaw((int16_t)imu.yaw_deg);

        imu_led_update(imu.yaw_deg);

        d3_led_update(ax_mps2); //is ax_mps2 correct??
        // Print about once per second instead of every 10 ms
        print_count++;
        if (print_count >= 100) {
            print_count = 0;

            x_cm = (int16_t)(imu.x_m * 100.0f);
            yaw_deg_int = (int16_t)(imu.yaw_deg);
            vel = (int16_t)(imu.vx_mps * 100.0f);
            acc = (int16_t)(imu.ax_mps2 * 100.0f);

            UartPrintString("X = ");
           UartPrint_float(x_cm);
            UartPrintString(" cm   Yaw = ");
            UartPrint_float(yaw_deg_int);
            UartPrintString(" deg  Velocity = ");
             UartPrint_float(vel);
            UartPrintString(" cm/s  Acceleration = ");
            UartPrint_float(acc);

            UartAddNewLine();
        }

        _delay_ms(10);
    }
}
