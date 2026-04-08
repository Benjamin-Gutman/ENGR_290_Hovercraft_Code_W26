#include "state_machine.h"
#include <math.h>
#include <stdint.h>
#include "sensors.h"

/* ===================== GLOBAL STATE ===================== */

State state = STATE_STRAIGHT;
TurnDirection turn_dir = TURN_RIGHT;

float yaw_start = 0.0f;
float yaw_target = 0.0f;

/* ===================== TUNING CONSTANTS ===================== */

#define TURN_THRESHOLD          30.0f
#define SIDE_OPEN_THRESHOLD     40.0f
#define YAW_TOLERANCE            5.0f

/* Exit turn slightly before 180 so RECENTER can finish cleanly */
#define TURN_EXIT_ANGLE        165.0f

/* Require consecutive detections before entering PREPARE_TURN */
#define DETECT_COUNT_REQUIRED    2

/* ===================== INTERNAL STATE ===================== */

static uint8_t turn_detect_count = 0;

/* ===================== HELPERS ===================== */

static float wrap_angle_deg(float angle)
{
    while (angle > 180.0f)  angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

float angle_diff(float a, float b)
{
    return wrap_angle_deg(a - b);
}

void enter_state(State new_state)
{
    state = new_state;
}

/* ===================== MAIN STATE MACHINE ===================== */

void update_state(void)
{
    switch (state) {

        case STATE_STRAIGHT:
        {
            /* Detect either:
               - open space on the left
               - wall ahead */
            if ((left_cm > SIDE_OPEN_THRESHOLD) || (front_cm < TURN_THRESHOLD)) {
                if (turn_detect_count < DETECT_COUNT_REQUIRED) {
                    turn_detect_count++;
                }
            } else {
                turn_detect_count = 0;
            }

            if (turn_detect_count >= DETECT_COUNT_REQUIRED) {
                turn_detect_count = 0;
                enter_state(STATE_PREPARE_TURN);
            }

            break;
        }

        case STATE_PREPARE_TURN:
        {
            yaw_start = yaw_deg;

            /* With current sensor layout:
               - if left side is open, prefer left turn
               - otherwise turn right */
            if (left_cm > SIDE_OPEN_THRESHOLD) {
                turn_dir = TURN_LEFT;
                yaw_target = wrap_angle_deg(yaw_deg + 180.0f);
            } else {
                turn_dir = TURN_RIGHT;
                yaw_target = wrap_angle_deg(yaw_deg - 180.0f);
            }

            enter_state(STATE_TURN);
            break;
        }

        case STATE_TURN:
        {
            float turned = angle_diff(yaw_deg, yaw_start);

            if ((turn_dir == TURN_LEFT  && turned >=  TURN_EXIT_ANGLE) ||
                (turn_dir == TURN_RIGHT && turned <= -TURN_EXIT_ANGLE)) {
                enter_state(STATE_RECENTER);
            }

            break;
        }

        case STATE_RECENTER:
        {
            float error = angle_diff(yaw_target, yaw_deg);

            if (fabsf(error) < YAW_TOLERANCE) {
                yaw_target = yaw_deg;
                enter_state(STATE_STRAIGHT);
            }

            break;
        }
    }
}
