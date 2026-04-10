//This code handles the control logic of the hovercraft
//It reads the current state and sends the commands to hardware, according to desired state behaviour

//This is where it decides HOW the hovercraft moves
//PD control works as long as delay is consistent and ~10ms, if we want more precise we have to divide by dt = 10ms and tune again

//For SERVO CONTROL, logic here works IF:
// +angles must turn the craft left
// -angles must turn the craft right
//The logic above is switched, after testing servo, +85 turns clockwise (right)
//If not the case, invert angles in SERVO code OR change SERVO_LEFT/RIGHT in constants

#include "control.h"
#include "state_machine.h"
#include "servo.h" //For set function
#include "thrust.h" //For set function
#include "sensors.h"
#include "lift.h"
#include <math.h>

/*=================CONTROL CONSTANTS================*/
//To be tuned

//Servo angle limits in degrees
//IF LOGIC IS REVERSED, DO NOT CHANGE VARIABLES, change RIGHT WITH LEFT INSTEAD
#define SERVO_LEFT_MAX -80.0f
#define SERVO_RIGHT_MAX 80.0f //Switched logic
#define SERVO_CENTER 0.0f

//Thrust values (placeholders values, not sure what they should be)
#define THRUST_NORMAL 90
#define THRUST_PREPARE 100
#define THRUST_TURN_RIGHT 20
#define THRUST_TURN_LEFT 75
// #define THRUST_FAILSAFE 0


//PD gains for yaw correction
//Needs to be tuned after testing
#define KP_YAW  1.8f
#define KD_YAW  1.8f
//KD was 0.4f
//KP is Proportional Gain - how strong the correction
//KD is Derivative Gain - how much resistance (damping)

//Boolean to cut turn

/*FOR TESTING:
 *KP:
 * If KP Is too low, hovercraft will drift
 * If KP is too high, craft will oscillate
 *
 * If KD is too low, no damping, aggressive reaction, overshoots, oscillates
 * If KD is too high, delayed turns, too much damping

/*=========Memory for control==========*/
//Stores the previous yaw error for derivative
static float previous_yaw_error = 0.0f;


/*============HELPER FUNCTIONS===========*/


//This function just clamps the angles to match servo limits (-85 to +85)
//similar func exists in servo code, must be adjusted
static float clamp_servo(float angle) {

   if (angle > SERVO_RIGHT_MAX) {
        return SERVO_RIGHT_MAX;
    }
    if (angle < SERVO_LEFT_MAX) {
        return SERVO_LEFT_MAX;
    }

    return angle;
}


//PD CONTROLLER for yaw correction
//This function must compute the steering correction needed to reduce yaw error
//Takes the current yaw error, and converts it into a steering command
static float yaw_pd_control(float target_yaw, float current_yaw) {

    //Compute yaw error and wrap angle, using function angle_diff() from state_machine
    float error = angle_diff(target_yaw, current_yaw);

    //Derivative of error (current error - previous error)
    //Not diving by time yet, ASSUMING LOOP delay IS CONSISTENT and is short like ~10ms
    float derivative = (error - previous_yaw_error);
    //This gives you how much the error changed since the last loop
    //MULTIPLY DERIVATIVE BY 0.7f to smooth it out if it jitters

    //Compute PD output
    float output = -(KP_YAW * error) - (KD_YAW * derivative);
    //KP_YAW controls how strong reaction must be to error
    //KD_YAW controls how much damping
    //Changed to negative feedback

    previous_yaw_error = error; //To store error

    return clamp_servo(output);//Returns a servo angle command between -85 to +85
}


/*=================MAIN Control System================*/

//This function must be in main loop, it decides the current servo angle and thrust
//Gives command to thrust and servo directly
//DEPENDS ON CURRENT STATE
void apply_control(void) {

        static State prev_state = STATE_STRAIGHT;

        if (state != prev_state) {
         previous_yaw_error = 0.0f;
        prev_state = state;
        }
    //Default commands, these variables will be overwritten depending on the state
    //then, they will be sent as parameters in the setting servo and setting thrust functions
    float servo_command = SERVO_CENTER;
    int thrust_command = THRUST_NORMAL;
    //Must be careful for int and float: can be changed, depends on what the parameter type is for the SET functions

    switch (state) { //State is a global variable from state_machine.c

        /*--------------------------------------------------*/
        case STATE_STRAIGHT:
        /*--------------------------------------------------*/
        {
            //When in straight state:
            //Must move forward at normal speed
            // use PD to keep heading aligned to yaw_target

            servo_command = yaw_pd_control(yaw_target, yaw_deg);   
            thrust_command = THRUST_NORMAL;
            break;
            //I just realized, yaw_target is not computed yet in this state --Issue fixed. the new target is assigned in recenter state - SHOULD NOT BE REASSIGNED
        }

        /*--------------------------------------------------*/
        case STATE_PREPARE_TURN:
        /*--------------------------------------------------*/
        {
            //In prepare_turn state:
            //Must slow down before turning (thrust control)
            //Still use PD to keep it stabilized (servo control)

            //servo_command = yaw_pd_control(yaw_target, yaw_deg); //removed because it would prematurely go to +-180
            servo_command = SERVO_CENTER;
            thrust_command = THRUST_PREPARE;

            break;
        }

        /*--------------------------------------------------*/
        case STATE_TURN:
       /*--------------------------------------------------*/
        {
            //In Turn state:
            //Must turn strongly (servo control)
            //NO PD HERE
            //Depending on direction from turn_dir (computed in state_machine)
            //Must decrease thrust

            if (turn_dir == TURN_LEFT) { //Decides servo behaviour depending on direction
                servo_command = SERVO_LEFT_MAX;
                  thrust_command = THRUST_TURN_LEFT;
               
            }
            else {
                servo_command = SERVO_RIGHT_MAX;
               thrust_command = THRUST_TURN_RIGHT;
            } //This can be adjusted, depending on servo "left" and "right"

         
            break;
        }

        /*--------------------------------------------------*/
        case STATE_RECENTER:
        /*--------------------------------------------------*/
        {
            /*
             * In recenter state:
             * - the 180 turn is finished
             * - now use PD to align precisely with yaw_target
             * - keep thrust lower for better stability
             */
            //In recenter state:
            //the turn (180 deg) must be completed
            //use PD to align to target yaw (servo control)
            //lower thrust (fan control)

            servo_command = yaw_pd_control(yaw_target, yaw_deg);
            thrust_command = THRUST_PREPARE;

            break;
        }

        // /*--------------------------------------------------*/
        // case STATE_FAILSAFE:
        //     /*--------------------------------------------------*/
        // {
        //
        //     break;
        // }

    }
    //OUTSIDE OF SWITCH STATEMENTS
    //This is where the actual set functions are called.
    //the servo_command and thrust_command values are computed in the switch cases above THEN the functions are called onto them
    set_servo_angle(servo_command);
    set_thrust(thrust_command);
}
