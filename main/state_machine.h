#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================ ENUM DEFINITIONS ========================*/

typedef enum {
    STATE_STRAIGHT,        // Moving forward, stabilizing heading
    STATE_PREPARE_TURN,    // Slow down and decide the crossing direction
    STATE_TURN_1,          // First 90 degree turn into the crossing
    STATE_CROSS_STRAIGHT,  // Hold heading while traversing the crossing
    STATE_TURN_2,          // Second 90 degree turn to complete the maneuver
    STATE_STOP_RECENTER,   // Pause after the turn once the straight corridor is detected
    STATE_RECENTER,        // Stabilize on the final heading
    // STATE_FAILSAFE
} State;

typedef enum {
    TURN_LEFT,
    TURN_RIGHT
} TurnDirection;

/*================ GLOBAL VARIABLES =======================*/

extern State state;
extern TurnDirection turn_dir;

/*
 * Current desired heading.
 * During the maneuver this changes from:
 * straight -> first 90 degree target -> crossing straight target -> final target.
 */
extern float yaw_target;

/*=============Functions=================*/

void update_state(void);
void enter_state(State new_state);
float wrap_angle(float angle);
float angle_diff(float a, float b);

#ifdef __cplusplus
}
#endif

#endif
