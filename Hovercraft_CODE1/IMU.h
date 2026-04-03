#ifndef IMU_H
#define IMU_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

    typedef struct {
        int16_t accel_x;
        int16_t accel_y;
        int16_t accel_z;
        int16_t temp;
        int16_t gyro_x;
        int16_t gyro_y;
        int16_t gyro_z;
    } mpu6050_data_t;

    typedef enum {
        AFS_2G  = 0,
        AFS_4G  = 1,
        AFS_8G  = 2,
        AFS_16G = 3
    } accel_range_t;

    typedef enum {
        GFS_250DPS  = 0,
        GFS_500DPS  = 1,
        GFS_1000DPS = 2,
        GFS_2000DPS = 3
    } gyro_range_t;

    typedef struct {
        float ax_g;
        float ay_g;
        float az_g;

        float gx_dps;
        float gy_dps;
        float gz_dps;

        float roll_deg;
        float pitch_deg;
        float yaw_deg;

        float ax_bias_g;
        float ay_bias_g;
        float az_bias_g;

        float gx_bias_dps;
        float gy_bias_dps;
        float gz_bias_dps;

        float vx_mps;
        float x_m;

        float ax_linear_g;
        float ax_linear_mps2;

        float prev_ax_mps2;
        float prev_vx_mps;

        uint8_t initialized;

        float prev_ax_linear_g;  // For high-pass filter
        float ax_hp_filter;      // High-pass filter state
    } imu_state_t;

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

    void imu_reset_state(imu_state_t *imu);
    void imu_calibrate(imu_state_t *imu, uint16_t samples);
    uint8_t imu_update(imu_state_t *imu, float dt_s);
#ifdef __cplusplus
}
#endif
#endif