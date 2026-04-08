#define F_CPU 16000000UL
// CPU clock frequency = 16 MHz, used for delay and timing calculations

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <math.h>
#include "I2C.h"
#include "IMU.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define F_CPU 16000000UL
// MPU6050 register addresses
#define MPU_ADDR             0x68
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40
#define MPU6050_TEMP_OUT_H   0x41
#define MPU6050_TEMP_OUT_L   0x42
#define MPU6050_GYRO_XOUT_H  0x43
#define MPU6050_GYRO_XOUT_L  0x44
#define MPU6050_GYRO_YOUT_H  0x45
#define MPU6050_GYRO_YOUT_L  0x46
#define MPU6050_GYRO_ZOUT_H  0x47
#define MPU6050_GYRO_ZOUT_L  0x48
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_GYRO_CONFIG  0x1B
#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1A
#define MPU6050_INT_ENABLE   0x38

// Conversion factors used to transform raw sensor readings into physical units
static float accel_sensitivity = 16384.0f;  // Default accelerometer sensitivity for ±2g
static float gyro_sensitivity  = 131.0f;    // Default gyroscope sensitivity for ±250 deg/s

uint8_t mpu6050_init(void)
{
    uint8_t who;

    _delay_ms(100);

    who = mpu6050_who_am_i();
    if (who != 0x68 && who != 0x69) return 1;

    // Wake up the sensor
    I2C_write_reg(0x01, MPU6050_PWR_MGMT_1);

    _delay_ms(50);

    return 0;
}

uint8_t mpu6050_who_am_i(void)
{
    return I2C_read_reg(MPU6050_WHO_AM_I);
}

uint8_t mpu6050_set_accel_range(accel_range_t range)
{
    switch(range) {
        case AFS_2G:  accel_sensitivity = 16384.0f; break;
        case AFS_4G:  accel_sensitivity = 8192.0f;  break;
        case AFS_8G:  accel_sensitivity = 4096.0f;  break;
        case AFS_16G: accel_sensitivity = 2048.0f;  break;
        default: return 1;
    }

    I2C_write_reg((range << 3), MPU6050_ACCEL_CONFIG);
    return 0;
}

uint8_t mpu6050_set_gyro_range(gyro_range_t range)
{
    switch(range) {
        case GFS_250DPS:  gyro_sensitivity = 131.0f; break;
        case GFS_500DPS:  gyro_sensitivity = 65.5f;  break;
        case GFS_1000DPS: gyro_sensitivity = 32.8f;  break;
        case GFS_2000DPS: gyro_sensitivity = 16.4f;  break;
        default: return 1;
    }

    I2C_write_reg((range << 3), MPU6050_GYRO_CONFIG);
    return 0;
}

uint8_t mpu6050_set_dlpf(uint8_t dlpf_cfg)
{
    I2C_write_reg((dlpf_cfg & 0x07), MPU6050_CONFIG);
    return 0;
}

uint8_t mpu6050_set_sample_rate_div(uint8_t div)
{
    I2C_write_reg(div, MPU6050_SMPLRT_DIV);
    return 0;
}

uint8_t mpu6050_read_all(mpu6050_data_t *data)
{
    uint8_t buf[14];

    I2C_read_multiple_reg(MPU6050_ACCEL_XOUT_H, buf, 14);

    data->accel_x = (int16_t)((buf[0]  << 8) | buf[1]);
    data->accel_y = (int16_t)((buf[2]  << 8) | buf[3]);
    data->accel_z = (int16_t)((buf[4]  << 8) | buf[5]);
    data->temp    = (int16_t)((buf[6]  << 8) | buf[7]);
    data->gyro_x  = (int16_t)((buf[8]  << 8) | buf[9]);
    data->gyro_y  = (int16_t)((buf[10] << 8) | buf[11]);
    data->gyro_z  = (int16_t)((buf[12] << 8) | buf[13]);

    return 0;
}

float mpu6050_accel_to_g(int16_t raw)
{
    return ((float)raw) / accel_sensitivity;
}

float mpu6050_gyro_to_dps(int16_t raw)
{
    return ((float)raw) / gyro_sensitivity;
}

float mpu6050_temp_to_c(int16_t raw_temp)
{
    return ((float)raw_temp / 340.0f) + 36.53f;
}

void imu_reset_state(imu_state_t *imu)
{
    imu->ax_g = 0.0f;
    imu->ay_g = 0.0f;
    imu->az_g = 0.0f;

    imu->gx_dps = 0.0f;
    imu->gy_dps = 0.0f;
    imu->gz_dps = 0.0f;

    imu->roll_deg = 0.0f;
    imu->pitch_deg = 0.0f;
    imu->yaw_deg = 0.0f;

    imu->ax_bias_g = 0.0f;
    imu->ay_bias_g = 0.0f;
    imu->az_bias_g = 0.0f;

    imu->gx_bias_dps = 0.0f;
    imu->gy_bias_dps = 0.0f;
    imu->gz_bias_dps = 0.0f;

    imu->vx_mps = 0.0f;
    imu->x_m = 0.0f;

    imu->ax_linear_g = 0.0f;
    imu->ax_linear_mps2 = 0.0f;
    imu->prev_ax_mps2 = 0.0f;
    imu->prev_vx_mps = 0.0f;

    imu->initialized = 0;
}

void imu_calibrate(imu_state_t *imu, uint16_t samples)
{
    uint16_t i;
    mpu6050_data_t raw;

    float sum_ax = 0.0f;
    float sum_ay = 0.0f;
    float sum_az = 0.0f;
    float sum_gx = 0.0f;
    float sum_gy = 0.0f;
    float sum_gz = 0.0f;

    for (i = 0; i < samples; i++) {
        if (mpu6050_read_all(&raw) == 0) {
            sum_ax += mpu6050_accel_to_g(raw.accel_x);
            sum_ay += mpu6050_accel_to_g(raw.accel_y);
            sum_az += mpu6050_accel_to_g(raw.accel_z);

            sum_gx += mpu6050_gyro_to_dps(raw.gyro_x);
            sum_gy += mpu6050_gyro_to_dps(raw.gyro_y);
            sum_gz += mpu6050_gyro_to_dps(raw.gyro_z);
        }

        _delay_ms(2);
    }

    imu->ax_bias_g = sum_ax / samples;
    imu->ay_bias_g = sum_ay / samples;
    imu->az_bias_g = (sum_az / samples) - 1.0f;

    imu->gx_bias_dps = sum_gx / samples;
    imu->gy_bias_dps = sum_gy / samples;
    imu->gz_bias_dps = sum_gz / samples;
}



// Read new sensor data, remove offsets, estimate angles, and integrate motion along X
uint8_t imu_update(imu_state_t *imu, float dt_s)
{
    mpu6050_data_t raw;
    float roll_acc;
    float pitch_acc;
    float pitch_rad;
    float ax_linear_g;
    float ax_mps2;

    if (mpu6050_read_all(&raw) != 0) return 1;

    imu->ax_g = mpu6050_accel_to_g(raw.accel_x) - imu->ax_bias_g;
    imu->ay_g = mpu6050_accel_to_g(raw.accel_y) - imu->ay_bias_g;
    imu->az_g = mpu6050_accel_to_g(raw.accel_z) - imu->az_bias_g;

    imu->gx_dps = mpu6050_gyro_to_dps(raw.gyro_x) - imu->gx_bias_dps;
    imu->gy_dps = mpu6050_gyro_to_dps(raw.gyro_y) - imu->gy_bias_dps;
    imu->gz_dps = mpu6050_gyro_to_dps(raw.gyro_z) - imu->gz_bias_dps;

    roll_acc  = atan2f(imu->ay_g, imu->az_g) * 180.0f / M_PI;
    pitch_acc = atan2f(-imu->ax_g, sqrtf(imu->ay_g * imu->ay_g + imu->az_g * imu->az_g)) * 180.0f / M_PI;

    if (!imu->initialized) {
        imu->roll_deg = roll_acc;
        imu->pitch_deg = pitch_acc;
        imu->yaw_deg = 0.0f;
        imu->initialized = 1;
        imu->prev_ax_mps2 = 0.0f;
        imu->prev_vx_mps = 0.0f;

        // Initialize high-pass filter state
        imu->ax_hp_filter = 0.0f;
    }

    imu->roll_deg  = 0.98f * (imu->roll_deg  + imu->gx_dps * dt_s) + 0.02f * roll_acc;
    imu->pitch_deg = 0.98f * (imu->pitch_deg + imu->gy_dps * dt_s) + 0.02f * pitch_acc;

    imu->yaw_deg += imu->gz_dps * dt_s;
    if (imu->yaw_deg > 180.0f)  imu->yaw_deg -= 360.0f;
    if (imu->yaw_deg < -180.0f) imu->yaw_deg += 360.0f;

    pitch_rad = imu->pitch_deg * M_PI / 180.0f;

    ax_linear_g = imu->ax_g - sinf(pitch_rad);  // Add because we subtracted gravity in calibration

    // Apply high-pass filter to remove DC bias (0.1 Hz cutoff)
    float alpha = 0.996f; // For 100Hz sampling, 0.1Hz cutoff
    imu->ax_hp_filter = alpha * (imu->ax_hp_filter + ax_linear_g - imu->prev_ax_linear_g);
    imu->prev_ax_linear_g = ax_linear_g;
    ax_linear_g = imu->ax_hp_filter;

    // Consistent deadband
    const float ACCEL_DEADBAND = 0.015f;  // 30 mg
    const float VEL_DEADBAND = 0.01f;    // 1 cm/s

    if (fabsf(ax_linear_g) < ACCEL_DEADBAND) {
        ax_linear_g = 0.0f;
    }

    imu->ax_linear_g = ax_linear_g;
    ax_mps2 = ax_linear_g * 9.80665f;
    imu->ax_linear_mps2 = ax_mps2;

    // Detect if sensor is stationary
    uint8_t is_stationary = (fabsf(imu->gx_dps) < 1.0f &&
                             fabsf(imu->gy_dps) < 1.0f &&
                             fabsf(imu->gz_dps) < 1.0f &&
                             fabsf(ax_linear_g) < ACCEL_DEADBAND);

    if (is_stationary) {
        // Stronger damping when stationary
        imu->vx_mps *= 0.9f;
        if (fabsf(imu->vx_mps) < VEL_DEADBAND) {
            imu->vx_mps = 0.0f;
            imu->prev_vx_mps = 0.0f;
            imu->prev_ax_mps2 = 0.0f;
        }
    } else {
        // Trapezoidal integration for velocity
        imu->vx_mps += 0.5f * (imu->prev_ax_mps2 + ax_mps2) * dt_s;

        // Very light damping even when moving (prevents unbounded drift)
        imu->vx_mps *= 0.999f;

        // Trapezoidal integration for position
        imu->x_m += 0.5f * (imu->prev_vx_mps + imu->vx_mps) * dt_s;

        imu->prev_vx_mps = imu->vx_mps;
        imu->prev_ax_mps2 = ax_mps2;
    }

    return 0;
}
