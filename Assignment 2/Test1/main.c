{\rtf1\ansi\ansicpg1252\cocoartf2822
\cocoatextscaling0\cocoaplatform0{\fonttbl\f0\fswiss\fcharset0 Helvetica;}
{\colortbl;\red255\green255\blue255;}
{\*\expandedcolortbl;;}
\margl1440\margr1440\vieww11520\viewh8400\viewkind0
\pard\tx720\tx1440\tx2160\tx2880\tx3600\tx4320\tx5040\tx5760\tx6480\tx7200\tx7920\tx8640\pardirnatural\partightenfactor0

\f0\fs24 \cf0 #include <avr/io.h>\
#include <util/delay.h>\
#include <stdint.h>\
#include <math.h>\
\
#include "I2C_driver.h"\
#include "IMU_code.h"\
#include "servo_code.h"\
\
int main(void)\
\{\
    imu_state_t imu;\
    // Structure that stores all IMU data and computed orientation \
    //(roll, pitch, yaw)\
    //This is created in imu_code\
    \
    // Initialize communication with IMU\
    I2C_init();\
    //From I2C_driver\
\
    // Initialize MPU6050 sensor\
    mpu6050_init();\
    //From imu_code\
\
    // Reset IMU internal state\
    imu_reset_state(&imu);\
    //From imu_code\
\
    // Calibrate IMU \
    imu_calibrate(&imu, 500);\
    //From imu_code\
\
    // Initialize servo\
    servo_initialize();\
    //From servo_code\
\
    while (1)\
    \{\
        // Update IMU estimation\
        imu_update(&imu, 0.01f);\
	// Read new MPU6050 data and update roll, pitch, and yaw (dt \uc0\u8776  0.01 s = 10 ms)\
				\
        // Send yaw to servo\
        servo_set_from_yaw((int16_t)imu.yaw_deg); //Converts float to int\
\
        _delay_ms(10);\
    \}\
\}}