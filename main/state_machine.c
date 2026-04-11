// This contains the state logic of the hovercraft.
// This code decides WHAT the hovercraft should do next, not HOW to control it.

#include "state_machine.h"

#include <math.h>
#include <stdint.h>

#include "sensors.h"

/*================ Global state ===================*/

State state = STATE_STRAIGHT;
TurnDirection turn_dir = TURN_RIGHT;
float yaw_target = 0.0f;

static float final_yaw_target = 0.0f;
static uint16_t cross_straight_counter = 0;
static uint8_t can_left_turn = 1;
static uint8_t cross_accumulation = 0;
static uint16_t cross_lockout_counter = 255;
static uint16_t straight_recovery_counter = 0;
static uint8_t prepare_turn_counter = 0;
static uint8_t startup_guard_counter = 0;
static uint8_t front_turn_detect_counter = 0;
static uint8_t stop_recenter_counter = 0;
static uint8_t stop_recenter_wait_counter = 0;

/*================ Constants ==================*/

#define TURN_THRESHOLD 60.0f
#define SIDE_OPEN_THRESHOLD 48.0f
#define SIDE_WALL_RESET_THRESHOLD 35.0f

#define TURN_SEGMENT_ANGLE_DEG 90.0f

#define YAW_TOLERANCE_START 20.0f
#define TURN_FINISH_TOLERANCE 12.0f
#define RECENTER_TOLERANCE 10.0f

#define CROSS_STRAIGHT_MIN_LOOPS 8
#define CROSS_STRAIGHT_TARGET_LOOPS 20
#define SECOND_TURN_FRONT_TRIGGER_CM 38.0f
#define STRAIGHT_LINE_FRONT_CLEAR_CM 52.0f
#define STOP_RECENTER_HOLD_LOOPS 50
#define STOP_RECENTER_MAX_WAIT_LOOPS 70
#define PREPARE_BRAKE_MIN_LOOPS 50
#define PREPARE_BRAKE_MAX_LOOPS 60
#define STARTUP_GUARD_LOOPS 25
#define FRONT_TURN_CONFIRM_LOOPS 3
#define CROSS_POST_TURN_LOCKOUT_LOOPS 70
#define CROSS_REPEAT_RESET_LOOPS 90
#define CROSS_RESET_FRONT_CLEAR_CM 50.0f
#define MAX_CONSECUTIVE_CROSSINGS 2

/*============= Helper funcs ==============*/

float angle_diff(float a, float b)
{
    float diff = a - b;

    while (diff > 180.0f) {
        diff -= 360.0f;
    }
    while (diff < -180.0f) {
        diff += 360.0f;
    }

    return diff;
}

void enter_state(State new_state)
{
    state = new_state;
}

float wrap_angle(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }

    return angle;
}

static float signed_turn_angle(TurnDirection direction)
{
    if (direction == TURN_LEFT) {
        return TURN_SEGMENT_ANGLE_DEG;
    }

    return -TURN_SEGMENT_ANGLE_DEG;
}

static uint8_t heading_is_aligned(float target_yaw_deg, float tolerance_deg)
{
    return (fabs(angle_diff(target_yaw_deg, yaw_deg)) <= tolerance_deg);
}

static void start_crossing_sequence(TurnDirection direction)
{
    float first_turn_delta = signed_turn_angle(direction);
    float base_yaw = yaw_target;

    yaw_target = wrap_angle(base_yaw + first_turn_delta);
    final_yaw_target = wrap_angle(base_yaw + (2.0f * first_turn_delta));
    cross_straight_counter = 0;
}

static uint8_t crossing_detection_enabled(void)
{
    return (cross_lockout_counter >= CROSS_POST_TURN_LOCKOUT_LOOPS) &&
           (cross_accumulation < MAX_CONSECUTIVE_CROSSINGS);
}

static void update_crossing_memory(void)
{
    if (startup_guard_counter < STARTUP_GUARD_LOOPS) {
        startup_guard_counter++;
    }

    if (cross_lockout_counter < CROSS_POST_TURN_LOCKOUT_LOOPS) {
        cross_lockout_counter++;
    }

    if ((state == STATE_STRAIGHT) &&
        heading_is_aligned(yaw_target, RECENTER_TOLERANCE) &&
        (front_cm > CROSS_RESET_FRONT_CLEAR_CM) &&
        (left_cm < SIDE_OPEN_THRESHOLD)) {
        if (straight_recovery_counter < CROSS_REPEAT_RESET_LOOPS) {
            straight_recovery_counter++;
        }

        if (straight_recovery_counter >= CROSS_REPEAT_RESET_LOOPS) {
            cross_accumulation = 0;
        }
    } else {
        straight_recovery_counter = 0;
    }
}

static uint8_t front_turn_detected(void)
{
    if (front_cm <= TURN_THRESHOLD) {
        if (front_turn_detect_counter < FRONT_TURN_CONFIRM_LOOPS) {
            front_turn_detect_counter++;
        }
    } else {
        front_turn_detect_counter = 0;
    }

    return (front_turn_detect_counter >= FRONT_TURN_CONFIRM_LOOPS);
}

static void reset_turn_detect_counters(void)
{
    front_turn_detect_counter = 0;
}

static uint8_t straight_line_detected(void)
{
    return (front_cm >= STRAIGHT_LINE_FRONT_CLEAR_CM) &&
           (left_cm < SIDE_OPEN_THRESHOLD);
}

/*================= STATE MACHINE ========================*/

void update_state(void)
{
    update_crossing_memory();

    if (left_cm < SIDE_WALL_RESET_THRESHOLD) {
        can_left_turn = 1;
    }

    switch (state) {
        case STATE_STRAIGHT:
        {
            uint8_t left_cross_detected = ((left_cm > SIDE_OPEN_THRESHOLD) && (can_left_turn == 1));
            uint8_t front_cross_detected = front_turn_detected();

            if ((startup_guard_counter >= STARTUP_GUARD_LOOPS) &&
                crossing_detection_enabled() &&
                (left_cross_detected || front_cross_detected)) {
                if (heading_is_aligned(yaw_target, YAW_TOLERANCE_START)) {
                    if (left_cross_detected) {
                        turn_dir = TURN_LEFT;
                    } else {
                        turn_dir = TURN_RIGHT;
                    }
                    prepare_turn_counter = 0;
                    reset_turn_detect_counters();
                    enter_state(STATE_PREPARE_TURN);
                }
            }

            break;
        }

        case STATE_PREPARE_TURN:
        {
            prepare_turn_counter++;

            if ((prepare_turn_counter >= PREPARE_BRAKE_MIN_LOOPS) &&
                ((front_cm <= TURN_THRESHOLD) ||
                 (prepare_turn_counter >= PREPARE_BRAKE_MAX_LOOPS))) {
                prepare_turn_counter = 0;
                start_crossing_sequence(turn_dir);
                enter_state(STATE_TURN_1);
            }
            break;
        }

        case STATE_TURN_1:
        {
            if (heading_is_aligned(yaw_target, TURN_FINISH_TOLERANCE)) {
                cross_straight_counter = 0;
                enter_state(STATE_CROSS_STRAIGHT);
            }

            break;
        }

        case STATE_CROSS_STRAIGHT:
        {
            cross_straight_counter++;

            if (cross_straight_counter >= CROSS_STRAIGHT_MIN_LOOPS) {
                if ((front_cm < SECOND_TURN_FRONT_TRIGGER_CM) ||
                    (cross_straight_counter >= CROSS_STRAIGHT_TARGET_LOOPS)) {
                    yaw_target = final_yaw_target;
                    enter_state(STATE_TURN_2);
                }
            }

            break;
        }

        case STATE_TURN_2:
        {
            if (straight_line_detected() ||
                heading_is_aligned(yaw_target, TURN_FINISH_TOLERANCE)) {
                stop_recenter_counter = 0;
                stop_recenter_wait_counter = 0;
                enter_state(STATE_STOP_RECENTER);
            }

            break;
        }

        case STATE_STOP_RECENTER:
        {
            if (straight_line_detected()) {
                if (stop_recenter_counter < STOP_RECENTER_HOLD_LOOPS) {
                    stop_recenter_counter++;
                }
            } else if (stop_recenter_counter == 0) {
                stop_recenter_counter = 0;
            }

            if (stop_recenter_wait_counter < STOP_RECENTER_MAX_WAIT_LOOPS) {
                stop_recenter_wait_counter++;
            }

            if ((stop_recenter_counter >= STOP_RECENTER_HOLD_LOOPS) ||
                (stop_recenter_wait_counter >= STOP_RECENTER_MAX_WAIT_LOOPS)) {
                stop_recenter_counter = 0;
                stop_recenter_wait_counter = 0;
                enter_state(STATE_RECENTER);
            }

            break;
        }

        case STATE_RECENTER:
        {
            float error = angle_diff(yaw_target, yaw_deg);

            if (fabs(error) <= RECENTER_TOLERANCE) {
                if (cross_accumulation < 255) {
                    cross_accumulation++;
                }
                cross_lockout_counter = 0;
                straight_recovery_counter = 0;
                prepare_turn_counter = 0;
                reset_turn_detect_counters();
                stop_recenter_counter = 0;
                stop_recenter_wait_counter = 0;

                if (turn_dir == TURN_LEFT) {
                    can_left_turn = 0;
                } else {
                    can_left_turn = 1;
                }

                enter_state(STATE_STRAIGHT);
            }

            break;
        }

        default:
        {
            enter_state(STATE_STRAIGHT);
            break;
        }
    }
}
