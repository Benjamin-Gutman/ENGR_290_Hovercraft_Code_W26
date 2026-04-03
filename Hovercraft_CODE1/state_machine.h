#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdint.h>


/* ================ ENUM DEFINITIONS ========================*/

//States of the hovercraft
typedef enum {
    STATE_STRAIGHT,        // Moving forward, stabilizing heading
    STATE_PREPARE_TURN,    // Decide turn direction and initialize variables
    STATE_TURN,            // Performing 180° turn (two times 90°)
    STATE_RECENTER,        // Stabilizing after turn using yaw control
    // STATE_FAILSAFE         // Optional safety state (not sure if we want to use)
} State;


//Turn direction
typedef enum {
    TURN_LEFT,
    TURN_RIGHT
} TurnDirection;


/*================ GLOBAL VARIABLES =======================*/

//Current State of the hovercraft - USED BY control.c TO DETERMINE BEHAVIOUR
extern State state; //extern makes it global btw

//Direction of current turn
//Used in control.c to set servo angle (+85 or -85)
extern TurnDirection turn_dir;
//Determined in the PREPARE STATE

//Yaw at the beginning of a 90° turn
// Used internally to measure how much we have turned
extern float yaw_start;

//This is the final DESIRED yaw for a full 180 turn
//This is used in control.c for PD correction (in recenter state and straight state)
extern float yaw_target;

//This is a counter for how many 90 deg turns have been done
// 0 means not started
// 1 means first 90° done
// 2 means full 180° done
extern uint8_t turn_count;

/*=============Functions=================*/

//Called in main loop, updates the state
void update_state(void);

//Changes current state, helper function
void enter_state(State new_state);



//Computers wrapped angle diff in degrees
//Range [-180, +180]
float angle_diff(float a, float b); //ORDER MATTERS
//angle_diff(target, current) returns error for PD
//angle_diff(current, start) returns how much craft has turned


#endif