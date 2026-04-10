//SERVO CODE, with timer1 configuration
//
#include <avr/io.h>
#include <stdint.h>
#include "servo.h"

#define SERVO_MIN_ANGLE   (-85) //Min angle allowed for servo rotation
#define SERVO_MAX_ANGLE   (85)  //Max angle allowed for servo rotation

//Servo PWM pulse width values (Timer1 ticks)
//These values correspond to the pulse width sent to the servo

#define SERVO_MIN_OCR     1200  //1.0 ms pulse -> about -85 degrees
#define SERVO_CENTER_OCR  3000  //1.5 ms pulse -> center position
#define SERVO_MAX_OCR     4800  //2.0 ms pulse -> about +85 degrees

//Whatever value is written into OCR1A controls the servo position
//Timer1 generates the PWM signal and OCR1A defines the pulse width

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


//This function takes an angle in, clamps the angle, and sets the servo angle
void set_servo_angle(float angle_deg)
{
    float angle = angle_deg;

    //Angle clamping
    if (angle > SERVO_MAX_ANGLE) {
        angle = SERVO_MAX_ANGLE;
    }
    if (angle < SERVO_MIN_ANGLE) {
        angle = SERVO_MIN_ANGLE;
    }

    uint16_t ocr;

    // Linear mapping:
    // -85 deg -> 2000
    //   0 deg -> 3000
    // +85 deg -> 4000

    ocr = SERVO_MIN_OCR;

    //Not sure if direction is reversed, if it is reversed:
    //Uncomment this part:
    //angle = -angle

    //Linear interpolation between min and max servo pulse widths
    ocr = SERVO_MIN_OCR + ((angle + 85.0f) * (SERVO_MAX_OCR - SERVO_MIN_OCR)) / 170.0f;

    //Write the calculated value into OCR1A
    //OCR1A controls the PWM pulse width sent to the servo
    OCR1A = ocr;
}