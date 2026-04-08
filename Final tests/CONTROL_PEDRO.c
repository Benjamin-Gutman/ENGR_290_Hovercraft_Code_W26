#include "control.h"
#include "state_machine.h"
#include "servo.h"
#include "thrust.h"
#include "sensors.h"
#include <math.h>

/* ===================== CONTROL CONSTANTS ===================== */

#define SERVO_LEFT_MAX      85.0f
#define SERVO_RIGHT_MAX    -85.0f
#define SERVO_CENTER         0.0f

#define THRUST_NORMAL       70
#define THRUST_PREPARE      50
#define THRUST_TURN         40

/* PD gains using yaw error and measured yaw rate */
#define KP_YAW              1.20f
#define KD_YAW              0.18f

/* ===================== HELPERS ===================== */

static float clamp_servo(float angle)
{
    if (angle > SERVO_LEFT_MAX)  return SERVO_LEFT_MAX;
    if (angle < SERVO_RIGHT_MAX) return SERVO_RIGHT_MAX;
    return angle;
}

static float yaw_pd_control(float target_yaw, float current_yaw)
{
    float error = angle_diff(target_yaw, current_yaw);

    /* D term uses measured yaw rate directly.
       If the steering direction is reversed on hardware,
       invert servo direction in servo code or swap left/right limits. */
    float output = (KP_YAW * error) - (KD_YAW * yaw_rate_dps);

    return clamp_servo(output);
}

/* ===================== MAIN CONTROL ===================== */

void apply_control(void)
{
    float servo_command = SERVO_CENTER;
    uint8_t thrust_command = THRUST_NORMAL;

    switch (state) {

        case STATE_STRAIGHT:
            servo_command = yaw_pd_control(yaw_target, yaw_deg);
            thrust_command = THRUST_NORMAL;
            break;

        case STATE_PREPARE_TURN:
            servo_command = yaw_pd_control(yaw_target, yaw_deg);
            thrust_command = THRUST_PREPARE;
            break;

        case STATE_TURN:
            if (turn_dir == TURN_LEFT) {
                servo_command = SERVO_LEFT_MAX;
            } else {
                servo_command = SERVO_RIGHT_MAX;
            }
            thrust_command = THRUST_TURN;
            break;

        case STATE_RECENTER:
            servo_command = yaw_pd_control(yaw_target, yaw_deg);
            thrust_command = THRUST_PREPARE;
            break;
    }

    set_servo_angle(servo_command);
    set_thrust(thrust_command);
}
