#define F_CPU 16000000UL

#include <Arduino.h>
#include "sensors.h"
#include "IMU.h"

static unsigned long last_us = 0;

void setup()
{
    Serial.begin(115200);

    sensors_init();

    /* Override gyro range for test */
    mpu6050_set_gyro_range(GFS_500DPS);

    delay(300);

    last_us = micros();
}

void loop()
{
    unsigned long now_us = micros();
    float dt = (now_us - last_us) * 1e-6f;
    last_us = now_us;

    if (imu_update(&imu, dt) == 0) {
        Serial.print("gz_dps: ");
        Serial.print(imu.gz_dps, 3);

        Serial.print("   yaw_deg: ");
        Serial.println(imu.yaw_deg, 3);
    }

    delay(5);
}
