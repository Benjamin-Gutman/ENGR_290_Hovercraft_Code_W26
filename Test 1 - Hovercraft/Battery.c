// #include "Battery.h"
// #include "Thrust_Fan.h"

// #include <avr/io.h>
// #include <util/delay.h>

// void adc_init(void) {
//     // Select Vref = AVcc, left adjust result = 0
//     ADMUX = (1 << REFS0); // REFS1 = 0, REFS0 = 1 => AVcc
//     // Enable ADC, enable interrupt = 0, prescaler = 128 (16MHz/128 = 125kHz)
//     ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
// }

// uint16_t adc_read(uint8_t channel) {
//     // Select ADC channel (0-7)
//     ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
    
//     // Start conversion
//     ADCSRA |= (1 << ADSC);
    
//     // Wait for conversion to finish
//     while (ADCSRA & (1 << ADSC));
    
//     // Return ADC value
//     return ADC;
// }

// void battery_protection(void){
//   uint16_t val = adc_read(7);
//   if (val <= 12){
//     shutdown();
//   }
// }

// void shutdown(void){
//   set_off();
//   while(1);
// }