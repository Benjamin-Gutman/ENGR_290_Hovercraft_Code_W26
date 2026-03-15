//Code 1, without timer1 config
//For servo control, using yaw of IMU as input

#include <avr/io.h>
#include <stdint.h>

#define SERVO_MIN_ANGLE   (-85) //Min angle allowed
#define SERVO_MAX_ANGLE   (85)  //Max angle allowed

//Calculation:
//works in PWM
//OCR1A = 1.0 x 2500/20 = 125
#define SERVO_MIN_OCR     125   
// 1.0 ms (-90deg)
//OCR1A = 1.5 x 2500/20 = 188
#define SERVO_CENTER_OCR  188   
// 1.5 ms (center)
//OCR1A = 2.0 x 2500/20 = 250
#define SERVO_MAX_OCR     250   
// 2.0 ms (+90deg)

//So, the allowed values for OCR1A is 125 - 250 
//Calculated using the programming hint

//Whatever value is inputted in OCR1A directly controls the servo
//Using this formula OCR1A = a x 2500/20

static int16_t force_yaw(int16_t yaw) 
//This function takes the yaw from IMU 
//and forces the input yaw to the servo to be bounded by min and max allowed
{
    if (yaw > SERVO_MAX_ANGLE) return SERVO_MAX_ANGLE;
    if (yaw < SERVO_MIN_ANGLE) return SERVO_MIN_ANGLE;
    return yaw;
}

void servo_initialize(void) //Initiatlizes the servo to the ports
{
    // PB1 = OC1A = servo output on P9
    DDRB |= (1 << PB1);
		//set PB1 as output pin
		//From controller doc
		
    // Timer1 setup function should technically be already done with the rest of the code
    // for the 20 ms servo period
    
    //input to 0CR1A controls the servo
    OCR1A = SERVO_CENTER_OCR;  //By default, centered
}

void servo_set_from_yaw(int16_t yaw_deg) //In degrees
{
    int16_t yaw_forced;
    int32_t ocr;

    yaw_forced = force_yaw(yaw_deg); //This calls the function that forces the yaw to be within range

    // Linear mapping:
    // -85 deg -> 125
    //   0 deg -> 188
    // +85 deg -> 250
    //
    // ocr = 125 + (yaw + 85) * (250 - 125) / 170

    ocr = SERVO_MIN_OCR;
 //Linear interpolation
    ocr += ((int32_t)(yaw_forced + 85) * (SERVO_MAX_OCR - SERVO_MIN_OCR)) / 170;
//using formula above

    OCR1A = (uint16_t)ocr; //down scale to 16
}
