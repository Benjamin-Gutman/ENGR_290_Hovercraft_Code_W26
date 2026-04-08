#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    STATE_STRAIGHT,
    STATE_PREPARE_TURN,
    STATE_TURN,
    STATE_RECENTER
} State;

typedef enum {
    TURN_LEFT,
    TURN_RIGHT
} TurnDirection;

extern State state;
extern TurnDirection turn_dir;
extern float yaw_start;
extern float yaw_target;

void update_state(void);
void enter_state(State new_state);

/* Wrapped angle difference in degrees, range [-180, 180] */
float angle_diff(float a, float b);

#ifdef __cplusplus
}
#endif

#endif
