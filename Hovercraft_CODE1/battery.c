#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include "battery.h"

/* ===================== CONFIGURATION ===================== */

/* Measured external analog reference */
#define ADC_AREF_V              4.96f

/* Battery divider ADC channel */
#define BATTERY_ADC_CHANNEL     2   /* ADC2 / PC2 */

/* Calibrated divider gain:
   V_bat = V_adc * BATTERY_DIVIDER_GAIN

   Based on your measurements:
   - real battery voltage ≈ 8.33 V
   - ADC reading ≈ 550
   - AREF ≈ 4.96 V

   This gives a divider gain close to 3.12.
*/
#define BATTERY_DIVIDER_GAIN    3.12f

/* Number of ADC samples averaged */
#define BATTERY_SAMPLES         16

/* 2S LiPo thresholds */
#define BATTERY_FULL_V          8.40f
#define BATTERY_LOW_THRESHOLD_V 7.20f
#define BATTERY_EMPTY_V         6.80f

/* ===================== GLOBALS ===================== */

uint16_t battery_adc = 0;
float battery_voltage_v = 0.0f;
uint8_t battery_percent = 0;

/* ===================== ADC HELPERS ===================== */

static uint16_t adc_read_channel_blocking(uint8_t channel)
{
    /* Keep external AREF, only change MUX bits */
    ADMUX = (ADMUX & 0xF0) | (channel & 0x07);
    _delay_us(10);

    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC)) {}

    return ADC;
}

static uint16_t battery_read_average(uint8_t n_samples)
{
    uint32_t sum = 0;
    uint8_t i;

    for (i = 0; i < n_samples; i++) {
        sum += adc_read_channel_blocking(BATTERY_ADC_CHANNEL);
        _delay_ms(2);
    }

    return (uint16_t)(sum / n_samples);
}

/* ===================== PUBLIC FUNCTIONS ===================== */

void battery_init(void)
{
    /* Disable digital input buffer on ADC2 / PC2 */
    DIDR0 |= (1 << BATTERY_ADC_CHANNEL);
}

void battery_update(void)
{
    float v_adc;
    float v_bat;
    float p;

    battery_adc = battery_read_average(BATTERY_SAMPLES);

    /* Voltage at the ADC pin */
    v_adc = ((float)battery_adc * ADC_AREF_V) / 1023.0f;

    /* Reconstructed battery voltage */
    v_bat = v_adc * BATTERY_DIVIDER_GAIN;
    battery_voltage_v = v_bat;

    /* Linear percentage estimate between EMPTY and FULL */
    if (v_bat <= BATTERY_EMPTY_V) {
        battery_percent = 0;
    }
    else if (v_bat >= BATTERY_FULL_V) {
        battery_percent = 100;
    }
    else {
        p = (v_bat - BATTERY_EMPTY_V) / (BATTERY_FULL_V - BATTERY_EMPTY_V);
        battery_percent = (uint8_t)(p * 100.0f + 0.5f);
    }
}

uint8_t battery_is_low(void)
{
    return (battery_voltage_v <= BATTERY_LOW_THRESHOLD_V);
}
