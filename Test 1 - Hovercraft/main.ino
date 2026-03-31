
// //IMU attached to P7 or P19
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <math.h>

// #include "Auxilary_Functions.h"

// int main(void)
// {
//     imu_state_t imu;
//     uint16_t print_count = 0;
//     int16_t x_cm;
//     int16_t yaw_deg_int;
//     int16_t vel;
//     int16_t xacc;
//     int16_t yacc;
//     int16_t zacc;
//     int16_t  pitch_deg_int;
//     int16_t roll_deg_int;
//     int16_t last_time;

//     // Initialize communication with IMU
//     I2C_init();

//     // Initialize MPU6050 sensor
//     mpu6050_init();

//     // Apply IMU configuration before calibration
//     mpu6050_set_accel_range(AFS_16G);
//     mpu6050_set_gyro_range(GFS_250DPS);
//     mpu6050_set_dlpf(3);
//     mpu6050_set_sample_rate_div(9);

//     // Reset IMU internal state
//     imu_reset_state(&imu);

//     // Calibrate IMU while completely still
//     imu_calibrate(&imu, 500);

//     // Initialize servo
//     timer1_servo_init();

//     // Initialize UART
//     UartInitialize();

//     d3_led_init();
//     imu_led_init();

//     TCCR1B |= (1<<CS11);
//     last_time = TCNT1;
//     while (1)
//     {
//         // Update IMU estimation

//         uint16_t now = TCNT1;
//         uint16_t diff = now - last_time;
//         last_time = now;
//         float dt = diff * 0.5e-6;
//         imu_update(&imu, dt);

//         // Send yaw to servo
//         servo_set_from_yaw((int16_t)imu.yaw_deg);
//         imu_led_update(imu.yaw_deg);
//         d3_led_update(imu.ax_linear_mps2);
//         // Print about once per second instead of every 10 ms
//         print_count++;
//         if (print_count >= 100) {
//             print_count = 0;

//             x_cm = (int16_t)(imu.x_m * 100.0f);
//             yaw_deg_int = (int16_t)(imu.yaw_deg);
//             vel = (int16_t)(imu.vx_mps * 100.0f);
//             xacc = (int16_t)(imu.ax_g * 100.0f);
//             yacc = (int16_t)(imu.ay_g * 100.0f);
//             zacc = (int16_t)(imu.az_g * 100.0f);
//             pitch_deg_int = (int16_t)(imu.pitch_deg);
//             roll_deg_int = (int16_t)(imu.roll_deg);


//             UartPrintString("X = ");
//            UartPrint_float(x_cm);
//             UartPrintString(" cm   Yaw = ");
//             UartPrint_float(yaw_deg_int);
//             UartPrintString(" deg  Pitch ");
//              UartPrint_float(pitch_deg_int);
//              UartPrintString(" deg  roll ");
//             UartPrint_float(roll_deg_int);
//             UartPrintString(" deg  X_acc = ");
//             UartPrint_float(xacc);
//             UartPrintString(" g  Y_acc = ");
//             UartPrint_float(yacc);
//             UartPrintString(" g  Z_acc = ");
//             UartPrint_float(zacc);
//             UartPrintString(" g");


//             UartAddNewLine();
//         }

//         _delay_ms(10);
//     }
// }


#include "Thrust_Fan.h"
#include "Battery.h"

int main(void){
  fan_initialization();
  while(1){
    set_max_speed();
  }
}