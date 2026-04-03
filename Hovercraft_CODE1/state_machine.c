//This contains the state logic of the hovercraft
//This code decided WHAT the hovercraft should do next, NO CONTROL

//Angle logic is correct assuming turning left INCREASES YAW and turning righ DECREASES YAW --need to check this
//

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
float yaw_start = 0.0f;  //This is the yaw when a turn begins
float yaw_target = 0.0f; //This is the DESIRED yaw after a turn

//Tracking turns ( 2 step 90 degree turn)
uint8_t turn_count = 0;

/*================ Constants ==================*/
//These are most of the threshold, to be tuned and fixed later

#define TURN_THRESHOLD 30.0f //cm
//This is the front sensor reading to detect an upcoming turn
#define SIDE_OPEN_THRESHOLD 40.0f //cm
//This is the gap, this is if left side is open
#define YAW_TOLERANCE 5.0f //deg
//Acceptable tolerance, just put 5 degrees for  now

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
    state = new_state;
}


/*=================STATE MACHINE========================*/

//This function is supposed to be run in the main loop,
//It updates the state based on sensor inputs

void update_state(void) {

    switch (state) {

        /*------------------*/
        case STATE_STRAIGHT:
        /*-----------------*/
        {
            //Detects upcoming turn using the front sensor:
            if (left_cm > SIDE_OPEN_THRESHOLD || front_cm < TURN_THRESHOLD) {

                enter_state(STATE_PREPARE_TURN);

            } //This account for LAST EXIT and any turn
            break;
        }


        /*------------------*/
        case STATE_PREPARE_TURN: //This is the state where direction is determined
        /*-----------------*/ {
            //Save yaw at the start of a turn
            yaw_start = yaw_deg;
            turn_count = 0; //Reset turn tracker

            //Decide direction using left sensor:
            if (left_cm > SIDE_OPEN_THRESHOLD) {
                turn_dir = TURN_LEFT;
            }
            else {
                turn_dir = TURN_RIGHT;
            }

            //This sets the final desired yaw, after a turn
            if (turn_dir == TURN_LEFT) {
                yaw_target = yaw_deg + 180.0f;
            }
            else{
                yaw_target = yaw_deg - 180.0f;
             }

            //GO to turn state directly (we can add a delay here)
            enter_state(STATE_TURN);
            break;
        }


        /*------------------*/
        case STATE_TURN:
            /*-----------------*/
        {
            // difference = how much we have rotated since yaw_start (in degs)
            float difference = angle_diff(yaw_deg, yaw_start);

            //LEFT TURN:
            if (turn_dir == TURN_LEFT && difference >= 90.0f) {

                turn_count++;
                if (turn_count < 2)
                {
                    // Start second 90°
                    yaw_start = yaw_deg;

                }
                else
                {
                    // Full 180° completed
                    enter_state(STATE_RECENTER);
                }
            } //This breaks down a 180 deg turn into two 90deg turns

            //RIGHT TURN:
            else if (turn_dir == TURN_RIGHT && difference <= -90.0f)
            {
                turn_count++;

                if (turn_count < 2)
                {
                    yaw_start = yaw_deg;
                }
                else
                {
                    enter_state(STATE_RECENTER);
                }
            }

            break;
        }


        /*------------------*/
        case STATE_RECENTER:
            /*-----------------*/
        {
            //Check if aligned with target heading:
            float error = angle_diff(yaw_deg, yaw_target); //Calculates how much correction we need, in between a turn

            if (fabs(error) < YAW_TOLERANCE) {

                yaw_target = yaw_deg; //the new heading target is the yaw that it is at right now, assuming its recentered correctly - very important for PD controller

                enter_state(STATE_STRAIGHT);
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

