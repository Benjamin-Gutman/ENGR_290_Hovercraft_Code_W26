#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <math.h>

#include "sensors.h"
#include "I2C.h"
#include "IMU.h"

/* ===================== CONFIGURATION ===================== */

/* Measured external analog reference voltage */
#define ADC_AREF_V          4.64f

/* ADC channel assignment
   Adjust these if your wiring is different */
#define LEFT_ADC_CHANNEL    0   /* ADC0 / PC0 */
#define FRONT_ADC_CHANNEL   1   /* ADC1 / PC1 */

/* Number of samples averaged for each IR reading */
#define IR_SAMPLES          32

/* Loop time used by imu_update() */
#define SENSOR_DT_S         0.010f

/* Useful IR measurement range */
#define IR_MIN_VALID_CM     7.0f
#define IR_MAX_VALID_CM     80.0f

/* ===================== GLOBAL VARIABLES ===================== */

float left_cm = 0.0f;
float front_cm = 200.0f;

uint16_t left_adc = 0;
uint16_t front_adc = 0;

float yaw_deg = 0.0f;
float yaw_rate_dps = 0.0f;

imu_state_t imu;

/* ===================== IR CALIBRATION ===================== */

typedef struct {
    uint16_t d_cm;
    uint16_t adc;
} CalPoint;

/* Left IR calibration table
   Replace later if you measure a separate table for the left sensor */
static const CalPoint cal_left[] = {
    {  7, 682 },
    {  8, 670 },
    { 10, 583 },
    { 15, 390 },
    { 20, 327 },
    { 25, 277 },
    { 30, 240 },
    { 35, 225 },
    { 40, 200 },
    { 45, 164 },
    { 50, 150 },
    { 55, 145 },
    { 60, 139 },
    { 80, 112 }
};

/* Front IR calibration table
   For now it is the same as the left sensor.
   Replace it if the front sensor behaves differently. */
static const CalPoint cal_front[] = {
    {  7, 682 },
    {  8, 670 },
    { 10, 583 },
    { 15, 390 },
    { 20, 327 },
    { 25, 277 },
    { 30, 240 },
    { 35, 225 },
    { 40, 200 },
    { 45, 164 },
    { 50, 150 },
    { 55, 145 },
    { 60, 139 },
    { 80, 112 }
};

#define CAL_LEFT_N   ((uint8_t)(sizeof(cal_left)  / sizeof(cal_left[0])))
#define CAL_FRONT_N  ((uint8_t)(sizeof(cal_front) / sizeof(cal_front[0])))

/* ===================== ADC FUNCTIONS ===================== */

static void adc_init_external_aref(void)
{
    /* Use external AREF:
       REFS1 = 0, REFS0 = 0 */
    ADMUX = 0x00;

    /* Enable ADC, prescaler 128 -> 125 kHz ADC clock */
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    ADCSRB = 0;

    /* Disable digital input buffers on used ADC pins */
    DIDR0 |= (1 << LEFT_ADC_CHANNEL) | (1 << FRONT_ADC_CHANNEL);
}

static uint16_t adc_read_channel_blocking(uint8_t channel)
{
    ADMUX = (channel & 0x07);
    _delay_us(10);

    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC)) {}

    return ADC;
}

static uint16_t read_ir_average(uint8_t channel, uint8_t n_samples)
{
    uint32_t sum = 0;
    uint8_t i;

    for (i = 0; i < n_samples; i++) {
        sum += adc_read_channel_blocking(channel);
        _delay_ms(2);
    }

    return (uint16_t)(sum / n_samples);
}

/* ===================== CALIBRATION HELPERS ===================== */

static float cm_from_adc_piecewise_extrap(uint16_t adc, const CalPoint *table, uint8_t n)
{
    uint8_t i;

    if (adc >= table[0].adc) {
        i = 0;
    }
    else if (adc <= table[n - 1].adc) {
        i = n - 2;
    }
    else {
        for (i = 0; i < (n - 1); i++) {
            float a0 = (float)table[i].adc;
            float a1 = (float)table[i + 1].adc;

            if (a0 >= a1) {
                if ((float)adc <= a0 && (float)adc >= a1) break;
            } else {
                if ((float)adc >= a0 && (float)adc <= a1) break;
            }
        }

        if (i >= (n - 1)) i = n - 2;
    }

    {
        float d0 = (float)table[i].d_cm;
        float d1 = (float)table[i + 1].d_cm;
        float a0 = (float)table[i].adc;
        float a1 = (float)table[i + 1].adc;
        float den = (a1 - a0);

        if (den == 0.0f) return d0;

        {
            float t = ((float)adc - a0) / den;
            return d0 + t * (d1 - d0);
        }
    }
}

uint8_t ir_is_valid(float cm)
{
    return (cm >= IR_MIN_VALID_CM && cm <= IR_MAX_VALID_CM);
}

/* ===================== PUBLIC FUNCTIONS ===================== */

void sensors_init(void)
{
    adc_init_external_aref();

    I2C_init();
    _delay_ms(100);

    mpu6050_init();
    mpu6050_set_accel_range(AFS_2G);
    mpu6050_set_gyro_range(GFS_250DPS);
    mpu6050_set_dlpf(3);
    mpu6050_set_sample_rate_div(7);

    imu_reset_state(&imu);
    imu_calibrate(&imu, 500);

    _delay_ms(100);
}

void update_sensors(void)
{
    /* Read IR sensors */
    left_adc  = read_ir_average(LEFT_ADC_CHANNEL, IR_SAMPLES);
    front_adc = read_ir_average(FRONT_ADC_CHANNEL, IR_SAMPLES);

    /* Convert ADC to distance */
    left_cm  = cm_from_adc_piecewise_extrap(left_adc,  cal_left,  CAL_LEFT_N);
    front_cm = cm_from_adc_piecewise_extrap(front_adc, cal_front, CAL_FRONT_N);

    /* Update IMU */
    if (imu_update(&imu, SENSOR_DT_S) == 0) {
        yaw_deg = imu.yaw_deg;
        yaw_rate_dps = imu.gz_dps;
    }
}
