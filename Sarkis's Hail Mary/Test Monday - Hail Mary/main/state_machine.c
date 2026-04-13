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

#define YAW_TOLERANCE_TURN 10.0f //deg
//Acceptable tolerance, tolerance for completing a turn

#define YAW_TOLERANCE_RECENTER 9.5f
//Acceptable tolerance to go back to straight after turning

#define INBETWEEN_COUNT 100.0f


#define SLOW_DOWN 400.0f //Slow down timer for straight state


int CAN_LEFT_TURN = 1;
int CAN_RIGHT_TURN = 1;
static uint8_t prepare_counter = 0;
static uint16_t state_counter = 0; //For straight state
static uint16_t inbetween = 0;

int slow_mode = 0;

static uint8_t recenter_count = 0;

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
    } if (new_state == STATE_RECENTER) {
        recenter_count = 0;  
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
    
    switch (state) {
        /*------------------*/
        case STATE_STRAIGHT:
            /*-----------------*/
        {
            state_counter ++;

            if (state_counter > SLOW_DOWN) {
                slow_mode = 1;
            } else {
                slow_mode = 0;
            }

            //Detects upcoming turn using the front sensor:
            if ((left_cm > SIDE_OPEN_THRESHOLD && CAN_LEFT_TURN == 1) || (front_cm < TURN_THRESHOLD)) {

                if (fabs(angle_diff(yaw_target, yaw_deg)) < YAW_TOLERANCE_STRAIGHT){

                    state_counter = 0;
                    enter_state(STATE_PREPARE_TURN);

                }

            } //This account for LAST EXIT and any turn
            break;
        }


        /*------------------*/
        case STATE_PREPARE_TURN: //This is the state where direction is determined
            /*-----------------*/ {
            //Save yaw at the start of a turn

            if(prepare_counter == 0){
                //Decide direction using left sensor:
                if ((left_cm > SIDE_OPEN_THRESHOLD) && CAN_LEFT_TURN ) {
                    turn_dir = TURN_LEFT;
                } else if (CAN_RIGHT_TURN){
                    turn_dir = TURN_RIGHT;
                }

            
            //This sets the final desired yaw, after a turn
                if (turn_dir == TURN_LEFT) {
                    yaw_target = wrap_angle(yaw_target + 90.0f);
                } else {
                    yaw_target = wrap_angle(yaw_target - 90.0f);
                }

                yaw_prev = yaw_deg;
                yaw_accum = 0.0f;
            }


            if (prepare_counter < 70) {
                prepare_counter++;
                break;   // STAY in PREPARE (DO NOT TRANSITION)
                }

            enter_state(STATE_TURN1);
            break;
        }


        /*------------------*/
        case STATE_TURN1:
            /*-----------------*/
        {
            // difference = how much we have rotated since yaw_start (in degs)
            float delta = angle_diff(yaw_deg, yaw_prev);

           //RANDOM SPIKE IMU GLITCH PROTECTION
            // if (fabs(delta) > 20.0f) {
            //  delta = 0.0f;
            // }

            if (turn_dir == TURN_LEFT) {
                if (delta > 0) yaw_accum += delta;
            } else { // TURN_RIGHT
                if (delta < 0) yaw_accum += delta;
            }

            yaw_prev = yaw_deg;


           if (turn_dir == TURN_LEFT && yaw_accum >= (90.0f - YAW_TOLERANCE_TURN)) {
                if (turn_dir == TURN_LEFT) {
                CAN_LEFT_TURN = 0; // disable future left turns
                CAN_RIGHT_TURN = 1;
              }else {
                CAN_RIGHT_TURN = 0;
                CAN_LEFT_TURN = 1;
              }
               enter_state(STATE_STRAIGHT_INBETWEEN);
            }

            
        if (turn_dir == TURN_RIGHT && yaw_accum <= -(90.0f - YAW_TOLERANCE_TURN)) {
            if (turn_dir == TURN_LEFT) {
                CAN_LEFT_TURN = 0; // disable future left turns
                CAN_RIGHT_TURN = 1;
              }else {
                CAN_RIGHT_TURN = 0;
                CAN_LEFT_TURN = 1;
              }
               enter_state(STATE_STRAIGHT_INBETWEEN);


        }

 

            

            break;
        }

        case STATE_STRAIGHT_INBETWEEN: 
            /*-----------------*/ {
            inbetween++;

            if(inbetween == 1){

                if (turn_dir == TURN_LEFT){

                    yaw_target = wrap_angle(yaw_target + 90.0f);

                }
                else{

                    yaw_target = wrap_angle(yaw_target - 90.0f);

                }
            }
            
            if(inbetween > INBETWEEN_COUNT){

                yaw_prev = yaw_deg;
                yaw_accum = 0.0f;
                inbetween = 0;
                enter_state(STATE_TURN2);
                
            }

            break;
        }

         case STATE_TURN2:
            /*-----------------*/
        {
            // difference = how much we have rotated since yaw_start (in degs)
            float delta = angle_diff(yaw_deg, yaw_prev);

            //RANDOM SPIKE IMU GLITCH PROTECTION
            // if (fabs(delta) > 20.0f) {
            //  delta = 0.0f;
            // }

            if (turn_dir == TURN_LEFT) {
                if (delta > 0) yaw_accum += delta;
            } else { // TURN_RIGHT
                if (delta < 0) yaw_accum += delta;
            }

            yaw_prev = yaw_deg;


            if (turn_dir == TURN_LEFT && yaw_accum >= (90.0f - YAW_TOLERANCE_TURN)) {
                enter_state(STATE_RECENTER);
            }

            if (turn_dir == TURN_RIGHT && yaw_accum <= -(90.0f - YAW_TOLERANCE_TURN)) {
                enter_state(STATE_RECENTER);
            }

            break;
        }

        /*------------------*/
        case STATE_RECENTER:
            /*-----------------*/
        {
            recenter_count++;
            if(recenter_count < 80){
                break;
            }

            //Check if aligned with target heading:
            float error = angle_diff(yaw_target, yaw_deg); //Calculates how much correction we need, in between a turn
            

            if (fabs(error) < YAW_TOLERANCE_RECENTER) {

                enter_state(STATE_STRAIGHT);
            }

            break;
        }

    }
}
