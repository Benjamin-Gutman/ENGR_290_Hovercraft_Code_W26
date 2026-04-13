//This contains the state logic of the hovercraft
//This code decided WHAT the hovercraft should do next, NO CONTROL

//Angle logic is correct assuming turning left(CCW) INCREASES YAW and turning right(CW) DECREASES YAW --need to check this
//NEED TO CHECK!

#include "state_machine.h"
#include <stdint.h>
#include <math.h>
#include "sensors.h"

/*================Global state===================*/

//Current state:

State state = STATE_STRAIGHT; //Default state, doesn't matter

//Direction of the upcoming turn
TurnDirection turn_dir = TURN_RIGHT; //Default state, doesn't matter

//Yaw variables:
float yaw_start = 0.0f; //This is the yaw when a turn begins
float yaw_target = 0.0f; //This is the DESIRED yaw after a turn
static float yaw_accum = 0.0f;
static float yaw_prev = 0.0f;
/*================ Constants ==================*/
//These are most of the threshold, to be tuned and fixed later

#define TURN_THRESHOLD 60.0f //cm
//This is the front sensor reading to detect an upcoming turn

#define YAW_TOLERANCE_STRAIGHT 30.0f //deg
//Craft won't detect turns until it's locked on target, tolerance

#define SIDE_OPEN_THRESHOLD 48.0f //cm
//This is the gap, this is if left side is open

#define YAW_TOLERANCE_TURN 70.0f //deg
//Acceptable tolerance, tolerance for completing a turn
#define YAW_TOLERANCE_RECENTER 5.0f
//Acceptable tolerance to go back to straight after turning

int CAN_LEFT_TURN = 1;
int CAN_RIGHT_TURN = 1;
static uint8_t prepare_counter = 0;
int recenter_count = 0;
/*=============Helper funcs==============*/

//This function makes sure the result is always between -180 and +180, it's an angle wrapper
//Takes the difference of angle a - b, and returns converted value (range of -180 to +180)
float angle_diff(float a, float b) {
    float diff = a - b;

    while (diff > 180.0f) {
        diff -= 360.0f; //subtracts 360 until reaches within bound
    }
    while (diff < -180.0f) {
        diff += 360.0f; //adds 360 until reaches within bound
    }

    return diff;
}

//State changer
void enter_state(State new_state) {
     if (new_state == STATE_PREPARE_TURN) {
        prepare_counter = 0;   // RESET HERE
    }
    state = new_state;
}

float wrap_angle(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

/*=================STATE MACHINE========================*/

//This function is supposed to be run in the main loop,
//It updates the state based on sensor inputs

void update_state(void) {
    // Re-enable left turns when a wall is detected again
    // if (left_cm < SIDE_OPEN_THRESHOLD) {
    //   CAN_LEFT_TURN = 1;
    // }
    
    switch (state) {
        /*------------------*/
        case STATE_STRAIGHT:
            /*-----------------*/
        {
            //Detects upcoming turn using the front sensor:
            if ((left_cm > SIDE_OPEN_THRESHOLD && CAN_LEFT_TURN == 1) || (front_cm < TURN_THRESHOLD && CAN_RIGHT_TURN)) {

                if (fabs(angle_diff(yaw_target, yaw_deg)) < YAW_TOLERANCE_STRAIGHT){

                    enter_state(STATE_PREPARE_TURN);

                }

            } //This account for LAST EXIT and any turn
            break;
        }


        /*------------------*/
        case STATE_PREPARE_TURN: //This is the state where direction is determined
            /*-----------------*/ {
            //Save yaw at the start of a turn
            
                // if (prepare_counter < 70) {
                // prepare_counter++;
                // break;   // STAY in PREPARE (DO NOT TRANSITION)
                // }

            //Decide direction using left sensor:
            if (left_cm > SIDE_OPEN_THRESHOLD ) {
                turn_dir = TURN_LEFT;
            } else {
                turn_dir = TURN_RIGHT;
            }

            //This sets the final desired yaw, after a turn
            if (turn_dir == TURN_LEFT) {
                //yaw_target = wrap_angle(yaw_deg + 180.0f);
                yaw_target = wrap_angle(yaw_target + 180.0f);
            } else {
                //yaw_target = wrap_angle(yaw_deg - 180.0f);
                yaw_target = wrap_angle(yaw_target - 180.0f);
            }
            yaw_prev = yaw_deg;
            yaw_accum = 0.0f;
            //GO to turn state directly (we can add a delay here) DO NOT ADD _MS_DELAY HERE!!! only add a counter if necessary
            enter_state(STATE_TURN);
            break;
        }


        /*------------------*/
        case STATE_TURN:
            /*-----------------*/
        {
            // difference = how much we have rotated since yaw_start (in degs)
            float delta = angle_diff(yaw_deg, yaw_prev);
            yaw_accum += delta;
            yaw_prev = yaw_deg;

            

            //LEFT TURN:
            // if (turn_dir == TURN_LEFT ) {
            //          if (yaw_accum >= 180.0f) {
            //         enter_state(STATE_RECENTER);
            //          }
            //     }

            // //RIGHT TURN:
            // else if (turn_dir == TURN_RIGHT) {
            //         if (yaw_accum <= -180.0f) {
            //         enter_state(STATE_RECENTER);
            //         }
            // }

            float abs_turn = fabs(yaw_accum);

            if (abs_turn >= (180.0f - YAW_TOLERANCE_TURN)) {

              if (turn_dir == TURN_LEFT) {
                CAN_LEFT_TURN = 0; // disable future left turns
                CAN_RIGHT_TURN = 1;
              }else {
                CAN_RIGHT_TURN = 0;
                CAN_LEFT_TURN = 1;
              }

              enter_state(STATE_RECENTER);
            }

            break;
        }


        /*------------------*/
        case STATE_RECENTER:
            /*-----------------*/
        {
            recenter_count++;
            if(recenter_count >80){

            //Check if aligned with target heading:
              float error = angle_diff(yaw_target, yaw_deg); //Calculates how much correction we need, in between a turn
            //MAKE SURE THE ORDER ABOVE IS CORRECT AFTER TESTING

            // THIS allows immediate turn if a gap to the left is detected (at exit phase)
            //Uncomment this if necessary -- must test first

            // if (left_cm > SIDE_OPEN_THRESHOLD) {
            //     enter_state(STATE_PREPARE_TURN);
            //     break;
            // }
            //This might break logic

              if (fabs(error) < YAW_TOLERANCE_RECENTER) {
               // yaw_target = yaw_deg;
                //Changed, the yaw_target is the locked to either 0 or -180

                  enter_state(STATE_STRAIGHT);
              }
              recenter_count = 0;
            }
            break;
        }

        // /*------------------*/
        // case STATE_FAILSAFE:
        //     /*-----------------*/
        // {
        //     break;
        // } //not used yet
    }
}