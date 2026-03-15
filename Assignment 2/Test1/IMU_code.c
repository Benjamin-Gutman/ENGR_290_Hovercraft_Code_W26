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
