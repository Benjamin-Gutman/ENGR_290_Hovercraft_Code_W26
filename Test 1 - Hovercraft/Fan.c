#define F_CPU 16000000UL
#include "Thrust_Fan.h"
#include <avr/io.h>

void fan_initialization(void){
  DDRD |= (1<<thrust);

  // Timer2 Fast PWM, output on OC2A
  TCCR0A = 0;
  TCCR0B = 0;

  // Fast PWM mode: WGM21 = 1, WGM20 = 1
  TCCR0A |= (1 << WGM01) | (1 << WGM00);

  // Non-inverting PWM on OC2A
  TCCR0A |= (1 << COM0A1);

  // Prescaler = 64
  TCCR0B |= (1 << CS01) | (1 << CS00);

  // Start with LED OFF
  OCR0A = 0;
}

void set_max_speed(void){
  OCR0A = 255;
}

void set_off(void){
  OCR0A = 0;
}