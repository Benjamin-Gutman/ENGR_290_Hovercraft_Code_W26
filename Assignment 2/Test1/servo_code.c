//Code 1.1, with timer1 configuration
//For servo control, using yaw of IMU as input
#include <avr/io.h>
#include <stdint.h>

#define SERVO_MIN_ANGLE   (-85) //Min angle allowed for servo rotation
#define SERVO_MAX_ANGLE   (85)  //Max angle allowed for servo rotation

//Servo PWM pulse width values (Timer1 ticks)
//These values correspond to the pulse width sent to the servo

#define SERVO_MIN_OCR     4000  //1.0 ms pulse -> about -85 degrees
#define SERVO_CENTER_OCR  3000  //1.5 ms pulse -> center position
#define SERVO_MAX_OCR     2000  //2.0 ms pulse -> about +85 degrees

//Whatever value is written into OCR1A controls the servo position
//Timer1 generates the PWM signal and OCR1A defines the pulse width

static int16_t force_yaw(int16_t yaw)
/** This function takes the yaw value from IMU
and forces the input yaw to stay within the allowed servo range
Also turns on LED_L if our of range **/
{
    if (yaw > SERVO_MAX_ANGLE) 
    {
        LEDL_PORT |= (1 << LEDL_BIT);   // LEDL on 
         return SERVO_MAX_ANGLE;
    }

    if (yaw < SERVO_MIN_ANGLE)
    { 
         LEDL_PORT |= (1 << LEDL_BIT);   // LEDL on
         return SERVO_MIN_ANGLE;
    }
    
    LEDL_PORT &= ~(1 << LEDL_BIT);      // LEDL off
    return yaw;  
}

void timer1_servo_init(void) //Timer1 configuration for servo PWM
{
    //PB1 = OC1A = servo output pin connected to P9
    DDRB |= (1 << PB1); 
    //set PB1 as output pin
    //From controller pin assignment document

    //Clear timer configuration registers
    TCCR1A = 0;
    TCCR1B = 0;

    //Configure Timer1 for Fast PWM mode using ICR1 as TOP
    TCCR1A |= (1 << COM1A1); //non-inverting PWM on OC1A
    TCCR1A |= (1 << WGM11);
    TCCR1B |= (1 << WGM13) | (1 << WGM12);

    //Set prescaler = 8
    TCCR1B |= (1 << CS11);

    //Clock = 16 MHz
    //With prescaler 8 -> timer frequency = 2 MHz
    //1 timer tick = 0.5 microseconds

    //Servo period = 20 ms
    //20 ms / 0.5 us = 40000 timer counts
    ICR1 = 40000;

    //Initialize servo at center position
    OCR1A = SERVO_CENTER_OCR;
}

void servo_set_from_yaw(int16_t yaw_deg)
{
    int16_t yaw_forced;
    int32_t ocr;

    yaw_forced = force_yaw(yaw_deg);

    // Linear mapping:
    // -85 deg -> 2000
    //   0 deg -> 3000
    // +85 deg -> 4000
    //Because the servo was moving opposite direction of IMU

    ocr = SERVO_MIN_OCR;

    //Linear interpolation between min and max servo pulse widths
    ocr += ((int32_t)(yaw_forced + 85) * (SERVO_MAX_OCR - SERVO_MIN_OCR)) / 170;

    //Write the calculated value into OCR1A
    //OCR1A controls the PWM pulse width sent to the servo
    OCR1A = (uint16_t)ocr;
}

//TO USE THIS function in MAIN():
//1. Call timer1_servo_init();
//2. In an infinite loop:
//Call servo_set_from_yaw(imu_yaw_deg)
//Where input is the yaw output from the IMU in degrees
