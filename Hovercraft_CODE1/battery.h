#ifndef BATTERY_H
#define BATTERY_H

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
  #endif
/* Raw ADC reading from the battery divider */
extern uint16_t battery_adc;

/* Estimated battery voltage in volts */
extern float battery_voltage_v;

/* Estimated battery percentage 0..100 */
extern uint8_t battery_percent;

/* Initialize battery monitor */
void battery_init(void);

/* Update battery ADC, voltage, and percentage */
void battery_update(void);

/* Returns 1 if the battery is below the low-voltage threshold */
uint8_t battery_is_low(void);

#endif
