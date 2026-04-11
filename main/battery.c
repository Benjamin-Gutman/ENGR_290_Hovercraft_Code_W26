#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include "battery.h"

/* ===================== CONFIGURATION ===================== */

/* Measured AVCC used as ADC reference */
#define BATTERY_ADC_REFERENCE_V     4.96f

/* Use the same reference mode as the IR sensors so both modules agree */
#define BATTERY_ADC_REFERENCE_BITS  (1 << REFS0)

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
#define BATTERY_DIVIDER_GAIN        3.12f

/* Number of ADC samples averaged */
#define BATTERY_SAMPLES             8
#define BATTERY_SAMPLE_DELAY_MS     1

/* 2S LiPo thresholds */
#define BATTERY_FULL_V              8.40f
#define BATTERY_EMPTY_V             6.80f

/* Use hysteresis so load spikes do not toggle low-battery state */
#define BATTERY_LOW_ENTER_V         7.10f
#define BATTERY_LOW_EXIT_V          7.30f
#define BATTERY_LOW_CONFIRM_SAMPLES 3

/* Small EMA filter to smooth motor/load noise without hiding trends */
#define BATTERY_FILTER_ALPHA        0.25f

/* ===================== GLOBALS ===================== */

uint16_t battery_adc = 0;
float battery_voltage_v = 0.0f;
uint8_t battery_percent = 0;

static uint8_t battery_low_state = 0;
static uint8_t battery_low_counter = 0;
static uint8_t battery_recover_counter = 0;
static uint8_t battery_filter_ready = 0;

typedef struct {
    uint8_t admux;
    uint8_t adcsra;
    uint8_t adcsrb;
} battery_adc_snapshot_t;

/* ===================== ADC HELPERS ===================== */

static uint16_t adc_read_channel_blocking(uint8_t channel)
{
    /* Preserve voltage reference bits, only update channel selection */
    ADMUX = (ADMUX & 0xF0) | (channel & 0x07);
    _delay_us(10);

    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC)) {}

    return ADC;
}

static void battery_adc_snapshot(battery_adc_snapshot_t *snapshot)
{
    snapshot->admux = ADMUX;
    snapshot->adcsra = ADCSRA;
    snapshot->adcsrb = ADCSRB;
}

static void battery_adc_restore(const battery_adc_snapshot_t *snapshot)
{
    ADMUX = snapshot->admux;
    ADCSRB = snapshot->adcsrb;
    ADCSRA = snapshot->adcsra;
}

static void battery_adc_prepare(void)
{
    if ((ADCSRA & (1 << ADEN)) == 0) {
        ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    }

    ADCSRB = 0;
    ADMUX = BATTERY_ADC_REFERENCE_BITS | (BATTERY_ADC_CHANNEL & 0x07);
    DIDR0 |= (1 << BATTERY_ADC_CHANNEL);

    /* Throw away the first conversion after a channel/reference change */
    adc_read_channel_blocking(BATTERY_ADC_CHANNEL);
}

static uint16_t battery_read_average(uint8_t n_samples)
{
    uint32_t sum = 0;
    uint8_t i;

    for (i = 0; i < n_samples; i++) {
        sum += adc_read_channel_blocking(BATTERY_ADC_CHANNEL);
        _delay_ms(BATTERY_SAMPLE_DELAY_MS);
    }

    return (uint16_t)(sum / n_samples);
}

static uint8_t battery_percent_from_voltage(float voltage_v)
{
    float p;

    if (voltage_v <= BATTERY_EMPTY_V) {
        return 0;
    }

    if (voltage_v >= BATTERY_FULL_V) {
        return 100;
    }

    p = (voltage_v - BATTERY_EMPTY_V) / (BATTERY_FULL_V - BATTERY_EMPTY_V);
    return (uint8_t)(p * 100.0f + 0.5f);
}

static void battery_update_low_state(float voltage_v)
{
    if (battery_low_state == 0) {
        battery_recover_counter = 0;

        if (voltage_v <= BATTERY_LOW_ENTER_V) {
            if (battery_low_counter < BATTERY_LOW_CONFIRM_SAMPLES) {
                battery_low_counter++;
            }

            if (battery_low_counter >= BATTERY_LOW_CONFIRM_SAMPLES) {
                battery_low_state = 1;
            }
        } else {
            battery_low_counter = 0;
        }
    } else {
        battery_low_counter = 0;

        if (voltage_v >= BATTERY_LOW_EXIT_V) {
            if (battery_recover_counter < BATTERY_LOW_CONFIRM_SAMPLES) {
                battery_recover_counter++;
            }

            if (battery_recover_counter >= BATTERY_LOW_CONFIRM_SAMPLES) {
                battery_low_state = 0;
            }
        } else {
            battery_recover_counter = 0;
        }
    }
}

/* ===================== PUBLIC FUNCTIONS ===================== */

void battery_init(void)
{
    /* Disable digital input buffer on ADC2 / PC2 */
    DIDR0 |= (1 << BATTERY_ADC_CHANNEL);

    battery_adc = 0;
    battery_voltage_v = 0.0f;
    battery_percent = 0;
    battery_low_state = 0;
    battery_low_counter = 0;
    battery_recover_counter = 0;
    battery_filter_ready = 0;
}

void battery_update(void)
{
    battery_adc_snapshot_t adc_snapshot;
    float v_adc;
    float v_bat;

    battery_adc_snapshot(&adc_snapshot);
    battery_adc_prepare();

    battery_adc = battery_read_average(BATTERY_SAMPLES);
    battery_adc_restore(&adc_snapshot);

    /* Voltage at the ADC pin */
    v_adc = ((float)battery_adc * BATTERY_ADC_REFERENCE_V) / 1023.0f;

    /* Reconstructed battery voltage */
    v_bat = v_adc * BATTERY_DIVIDER_GAIN;

    if (battery_filter_ready == 0) {
        battery_voltage_v = v_bat;
        battery_filter_ready = 1;
    } else {
        battery_voltage_v += BATTERY_FILTER_ALPHA * (v_bat - battery_voltage_v);
    }

    battery_percent = battery_percent_from_voltage(battery_voltage_v);
    battery_update_low_state(battery_voltage_v);
}

uint8_t battery_is_low(void)
{
    return battery_low_state;
}
