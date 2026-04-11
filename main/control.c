// This code handles the control logic of the hovercraft.
// It reads the current state and sends commands to hardware.

#include "control.h"

#include <math.h>

#include "lift.h"
#include "sensors.h"
#include "servo.h"
#include "state_machine.h"
#include "thrust.h"

/*================= CONTROL CONSTANTS ================*/

#define SERVO_LEFT_MAX -80.0f
#define SERVO_RIGHT_MAX 80.0f
#define SERVO_CENTER 0.0f

#define THRUST_NORMAL 92
#define THRUST_PREPARE 0
#define THRUST_CROSS 82
#define THRUST_STOP_RECENTER 0
#define THRUST_TURN_RIGHT 94
#define THRUST_TURN_LEFT 98
#define THRUST_RECENTER 36

#define LIFT_NORMAL 100
#define LIFT_STOPPED 0

#define KP_YAW 2.2f
#define KD_YAW 4.0f

#define KP_TURN 1.4f
#define TURN_SERVO_MIN_RIGHT 42.0f
#define TURN_SERVO_MIN_LEFT 45.0f
#define TURN_CENTER_WINDOW 2.0f

/*========= Memory for control ==========*/

static float previous_yaw_error = 0.0f;

/*============ Helper functions ===========*/

static float clamp_servo(float angle)
{
    if (angle > SERVO_RIGHT_MAX) {
        return SERVO_RIGHT_MAX;
    }
    if (angle < SERVO_LEFT_MAX) {
        return SERVO_LEFT_MAX;
    }

    return angle;
}

static float yaw_pd_control(float target_yaw, float current_yaw)
{
    float error = angle_diff(target_yaw, current_yaw);
    float derivative = (error - previous_yaw_error);
    float output = -(KP_YAW * error) - (KD_YAW * derivative);

    previous_yaw_error = error;

    return clamp_servo(output);
}

static float turn_servo_control(float target_yaw, float current_yaw)
{
    float error = angle_diff(target_yaw, current_yaw);
    float output;
    float min_turn_servo;

    if (fabs(error) <= TURN_CENTER_WINDOW) {
        return SERVO_CENTER;
    }

    output = -(KP_TURN * error);
    min_turn_servo = (turn_dir == TURN_LEFT) ? TURN_SERVO_MIN_LEFT : TURN_SERVO_MIN_RIGHT;

    if ((output > 0.0f) && (output < min_turn_servo)) {
        output = min_turn_servo;
    } else if ((output < 0.0f) && (output > -min_turn_servo)) {
        output = -min_turn_servo;
    }

    return clamp_servo(output);
}

static int turn_thrust_command(void)
{
    if (turn_dir == TURN_LEFT) {
        return THRUST_TURN_LEFT;
    }

    return THRUST_TURN_RIGHT;
}

/*================= MAIN Control System =================*/

void apply_control(void)
{
    static State prev_state = STATE_STRAIGHT;
    float servo_command = SERVO_CENTER;
    int thrust_command = THRUST_NORMAL;
    uint8_t lift_command = LIFT_NORMAL;

    if (state != prev_state) {
        previous_yaw_error = 0.0f;
        prev_state = state;
    }

    switch (state) {
        case STATE_STRAIGHT:
        {
            servo_command = yaw_pd_control(yaw_target, yaw_deg);
            thrust_command = THRUST_NORMAL;
            lift_command = LIFT_NORMAL;
            break;
        }

        case STATE_PREPARE_TURN:
        {
            servo_command = yaw_pd_control(yaw_target, yaw_deg);
            thrust_command = THRUST_PREPARE;
            lift_command = LIFT_STOPPED;
            break;
        }

        case STATE_TURN_1:
        case STATE_TURN_2:
        {
            servo_command = turn_servo_control(yaw_target, yaw_deg);
            thrust_command = turn_thrust_command();
            lift_command = LIFT_NORMAL;
            break;
        }

        case STATE_CROSS_STRAIGHT:
        {
            servo_command = yaw_pd_control(yaw_target, yaw_deg);
            thrust_command = THRUST_CROSS;
            lift_command = LIFT_NORMAL;
            break;
        }

        case STATE_STOP_RECENTER:
        {
            servo_command = SERVO_CENTER;
            thrust_command = THRUST_STOP_RECENTER;
            lift_command = LIFT_STOPPED;
            break;
        }

        case STATE_RECENTER:
        {
            servo_command = yaw_pd_control(yaw_target, yaw_deg);
            thrust_command = THRUST_RECENTER;
            lift_command = LIFT_NORMAL;
            break;
        }

        default:
        {
            servo_command = SERVO_CENTER;
            thrust_command = THRUST_PREPARE;
            lift_command = LIFT_STOPPED;
            break;
        }
    }

    set_servo_angle(servo_command);
    set_thrust(thrust_command);
    set_lift(lift_command);
}
