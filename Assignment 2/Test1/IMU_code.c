#define F_CPU 16000000UL 
// CPU clock frequency = 16 MHz, used for delay and timing calculations

#include <avr/io.h> // AVR register definitions (ports, timers, control registers, etc.)
#include <util/delay.h> // Delay functions such as _delay_ms() and _delay_us()
#include <stdint.h> // Fixed-width integer types like uint8_t, int16_t, uint32_t
#include <math.h> // Math functions such as sqrt(), atan2(), and fabs()
#include "I2C_driver.h" //Header for the I2C functions
#include "IMU_code.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif



// I2C is a two-wire serial communication protocol used to exchange data between the AVR and external devices such as the MPU6050

//MPU6050 registers-> Register addresses used to read data from and configure the MPU6050

#define MPU_ADDR        0x68 
// Default I2C address of the MPU6050
#define MPU6050_WHO_AM_I    0x75 
// Identification register used to verify the device
#define MPU6050_PWR_MGMT_1  0x6B  
// Power management register used to wake up and configure the sensor clock
#define MPU6050_ACCEL_XOUT_H 0x3B   
// Accelerometer X-axis high byte
#define MPU6050_ACCEL_XOUT_L 0x3C   
// Accelerometer X-axis low byte
#define MPU6050_ACCEL_YOUT_H 0x3D  
// Accelerometer Y-axis high byte
#define MPU6050_ACCEL_YOUT_L 0x3E   
// Accelerometer Y-axis low byte
#define MPU6050_ACCEL_ZOUT_H 0x3F   
// Accelerometer Z-axis high byte
#define MPU6050_ACCEL_ZOUT_L 0x40   
// Accelerometer Z-axis low byte
#define MPU6050_TEMP_OUT_H 0x41   
// Temperature sensor high byte
#define MPU6050_TEMP_OUT_L 0x42   
// Temperature sensor low byte
#define MPU6050_GYRO_XOUT_H 0x43   
// Gyroscope X-axis high byte
#define MPU6050_GYRO_XOUT_L 0x44   
// Gyroscope X-axis low byte
#define MPU6050_GYRO_YOUT_H 0x45   
// Gyroscope Y-axis high byte
#define MPU6050_GYRO_YOUT_L 0x46   
// Gyroscope Y-axis low byte
#define MPU6050_GYRO_ZOUT_H 0x47   
// Gyroscope Z-axis high byte
#define MPU6050_GYRO_ZOUT_L 0x48   
// Gyroscope Z-axis low byte
#define MPU6050_ACCEL_CONFIG 0x1C   
// Accelerometer configuration register (full-scale range)
#define MPU6050_GYRO_CONFIG 0x1B   
// Gyroscope configuration register (full-scale range)
#define MPU6050_SMPLRT_DIV 0x19   
// Sample rate divider register
#define MPU6050_CONFIG 0x1A   
// General configuration register, including the digital low-pass filter
#define MPU6050_INT_ENABLE 0x38   
// Interrupt enable register


// Structure used to store the raw sensor readings from the MPU6050
typedef struct {
    int16_t accel_x;   // Raw accelerometer reading along the X axis
    int16_t accel_y;   // Raw accelerometer reading along the Y axis
    int16_t accel_z;   // Raw accelerometer reading along the Z axis
    int16_t temp;      // Raw temperature sensor reading
    int16_t gyro_x;    // Raw gyroscope reading along the X axis
    int16_t gyro_y;    // Raw gyroscope reading along the Y axis
    int16_t gyro_z;    // Raw gyroscope reading along the Z axis
} mpu6050_data_t;

//Available full-scale range settings for the accelerometer
typedef enum {
    AFS_2G  = 0,   // Accelerometer range set to ±2g
    AFS_4G  = 1,   // Accelerometer range set to ±4g
    AFS_8G  = 2,   // Accelerometer range set to ±8g
    AFS_16G = 3    // Accelerometer range set to ±16g
} accel_range_t;

// Available full-scale range settings for the gyroscope
typedef enum {
    GFS_250DPS  = 0,   // Gyroscope range set to ±250 degrees per second
    GFS_500DPS  = 1,   // Gyroscope range set to ±500 degrees per second
    GFS_1000DPS = 2,   // Gyroscope range set to ±1000 degrees per second
    GFS_2000DPS = 3    // Gyroscope range set to ±2000 degrees per second
} gyro_range_t;

uint8_t mpu6050_init(void);
uint8_t mpu6050_who_am_i(void);
uint8_t mpu6050_set_accel_range(accel_range_t range);
uint8_t mpu6050_set_gyro_range(gyro_range_t range);
uint8_t mpu6050_set_dlpf(uint8_t dlpf_cfg);
uint8_t mpu6050_set_sample_rate_div(uint8_t div);
uint8_t mpu6050_read_all(mpu6050_data_t *data);

float mpu6050_accel_to_g(int16_t raw);
float mpu6050_gyro_to_dps(int16_t raw);
float mpu6050_temp_to_c(int16_t raw_temp);


// Conversion factors used to transform raw sensor readings into physical units
static float accel_sensitivity = 16384.0f;  // Default accelerometer sensitivity for ±2g
static float gyro_sensitivity  = 131.0f;    // Default gyroscope sensitivity for ±250 deg/s



// MPU6050 basic configuration and data reading functions
uint8_t mpu6050_init(void)
{
    uint8_t who;

    _delay_ms(100);

    who = mpu6050_who_am_i();
    if (who != 0x68 && who != 0x69) return 1;

    I2C_write_reg(0x01, MPU6050_PWR_MGMT_1);

    _delay_ms(100);

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


// Convert a raw accelerometer reading into acceleration in g
float mpu6050_accel_to_g(int16_t raw)
{
    return ((float)raw) / accel_sensitivity;
}

// Convert a raw gyroscope reading into angular velocity in degrees per second
float mpu6050_gyro_to_dps(int16_t raw)
{
    return ((float)raw) / gyro_sensitivity;
}

// Convert a raw temperature reading into degrees Celsius using the MPU6050 formula
float mpu6050_temp_to_c(int16_t raw_temp)
{
    return ((float)raw_temp / 340.0f) + 36.53f;
}

// Structure used to store processed IMU data, calibration offsets, and estimated motion state
typedef struct {
    float ax_g;          // Acceleration along X axis in g
    float ay_g;          // Acceleration along Y axis in g
    float az_g;          // Acceleration along Z axis in g

    float gx_dps;        // Angular velocity around X axis in deg/s
    float gy_dps;        // Angular velocity around Y axis in deg/s
    float gz_dps;        // Angular velocity around Z axis in deg/s

    float roll_deg;      // Estimated roll angle in degrees
    float pitch_deg;     // Estimated pitch angle in degrees
    float yaw_deg;       // Estimated yaw angle in degrees

    float ax_bias_g;     // Accelerometer X offset
    float ay_bias_g;     // Accelerometer Y offset
    float az_bias_g;     // Accelerometer Z offset

    float gx_bias_dps;   // Gyroscope X offset
    float gy_bias_dps;   // Gyroscope Y offset
    float gz_bias_dps;   // Gyroscope Z offset

    float vx_mps;        // Estimated velocity along X in m/s
    float x_m;           // Estimated position along X in meters

    uint8_t initialized; // Indicates whether the filter has been initialized
} imu_state_t;

void imu_reset_state(imu_state_t *imu);
void imu_calibrate(imu_state_t *imu, uint16_t samples);
uint8_t imu_update(imu_state_t *imu, float dt_s);

// Reset all processed IMU values, offsets, velocity, and position
void imu_reset_state(imu_state_t *imu)
{
    // Reset accelerometer values in g
    imu->ax_g = 0.0f;
    imu->ay_g = 0.0f;
    imu->az_g = 0.0f;

    // Reset gyroscope values in degrees per second
    imu->gx_dps = 0.0f;
    imu->gy_dps = 0.0f;
    imu->gz_dps = 0.0f;

    // Reset estimated orientation angles
    imu->roll_deg = 0.0f;
    imu->pitch_deg = 0.0f;
    imu->yaw_deg = 0.0f;

    // Reset accelerometer calibration offsets
    imu->ax_bias_g = 0.0f;
    imu->ay_bias_g = 0.0f;
    imu->az_bias_g = 0.0f;

    // Reset gyroscope calibration offsets
    imu->gx_bias_dps = 0.0f;
    imu->gy_bias_dps = 0.0f;
    imu->gz_bias_dps = 0.0f;

    // Reset estimated velocity and position along X
    imu->vx_mps = 0.0f;
    imu->x_m = 0.0f;

    // Mark the filter as not initialized yet
    imu->initialized = 0;
}

// Calibrate accelerometer and gyroscope offsets by averaging several samples while the IMU is stationary
void imu_calibrate(imu_state_t *imu, uint16_t samples)
{
    // Loop counter
    uint16_t i;

    // Structure used to store one raw reading from the MPU6050
    mpu6050_data_t raw;

    // Accumulators used to compute average offsets
    float sum_ax = 0.0f;
    float sum_ay = 0.0f;
    float sum_az = 0.0f;
    float sum_gx = 0.0f;
    float sum_gy = 0.0f;
    float sum_gz = 0.0f;

    // Collect and accumulate multiple sensor samples
    for (i = 0; i < samples; i++) {
        // Read one sample from the MPU6050 and continue only if the read is successful
        if (mpu6050_read_all(&raw) == 0) {
            // Convert raw accelerometer readings to g and accumulate them
            sum_ax += mpu6050_accel_to_g(raw.accel_x);
            sum_ay += mpu6050_accel_to_g(raw.accel_y);
            sum_az += mpu6050_accel_to_g(raw.accel_z);

            // Convert raw gyroscope readings to deg/s and accumulate them
            sum_gx += mpu6050_gyro_to_dps(raw.gyro_x);
            sum_gy += mpu6050_gyro_to_dps(raw.gyro_y);
            sum_gz += mpu6050_gyro_to_dps(raw.gyro_z);
        }

        // Small delay between samples to avoid reading too fast
        _delay_ms(2);
    }

    // Compute the average accelerometer offsets
    imu->ax_bias_g = sum_ax / samples;
    imu->ay_bias_g = sum_ay / samples;
    imu->az_bias_g = (sum_az / samples) - 1.0f;   // Assumes Z axis measures +1g at rest

    // Compute the average gyroscope offsets
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
    }

    imu->roll_deg  = 0.98f * (imu->roll_deg  + imu->gx_dps * dt_s) + 0.02f * roll_acc;
    imu->pitch_deg = 0.98f * (imu->pitch_deg + imu->gy_dps * dt_s) + 0.02f * pitch_acc;

    imu->yaw_deg += imu->gz_dps * dt_s;

    if (imu->yaw_deg > 180.0f)  imu->yaw_deg -= 360.0f;
    if (imu->yaw_deg < -180.0f) imu->yaw_deg += 360.0f;

    pitch_rad = imu->pitch_deg * M_PI / 180.0f;

    /* Remove gravity component from X acceleration */
    ax_linear_g = imu->ax_g + sinf(pitch_rad);

    /* Smaller deadband so slow motion is not ignored */
    if (fabsf(ax_linear_g) < 0.015f) {
        ax_linear_g = 0.0f;
    }

    imu->ax_linear_g = ax_linear_g;
    ax_mps2 = ax_linear_g * 9.80665f;
    imu->ax_linear_mps2 = ax_mps2;

    /* Only zero velocity if everything is quiet AND speed is already very small */
    if (fabsf(ax_linear_g) < 0.015f &&
        fabsf(imu->gx_dps) < 1.0f &&
        fabsf(imu->gy_dps) < 1.0f &&
        fabsf(imu->gz_dps) < 1.0f &&
        fabsf(imu->vx_mps) < 0.03f) {

        imu->vx_mps = 0.0f;
        imu->prev_vx_mps = 0.0f;
        imu->prev_ax_mps2 = 0.0f;
    } else {
        /* Trapezoidal integration for velocity */
        imu->vx_mps += 0.5f * (imu->prev_ax_mps2 + ax_mps2) * dt_s;

        /* Smaller velocity deadband */
        if (fabsf(imu->vx_mps) < 0.005f) {
            imu->vx_mps = 0.0f;
        }

        /* Trapezoidal integration for position */
        imu->x_m += 0.5f * (imu->prev_vx_mps + imu->vx_mps) * dt_s;

        imu->prev_vx_mps = imu->vx_mps;
        imu->prev_ax_mps2 = ax_mps2;
    }

    return 0;
}
