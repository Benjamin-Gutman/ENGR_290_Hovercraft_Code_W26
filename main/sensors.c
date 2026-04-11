 #define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <math.h>

#include "sensors.h"
#include "I2C.h"
#include "IMU.h"

/* ===================== CONFIGURATION ===================== */

#define LEFT_ADC_CHANNEL    1   /* ADC1 / PC1 */ //USE F25 sensor for left
#define FRONT_ADC_CHANNEL   0   /* ADC0 / PC0 */ //USE F17 sensor for FRONT

/* Reduced samples because the EMA filter handles the heavy lifting now */
#define IR_SAMPLES          10  

#define IR_MIN_VALID_CM     7.0f
#define IR_MAX_VALID_CM     65.0f

/* ===================== GLOBAL VARIABLES ===================== */
/* These map directly to the 'extern' declarations in sensors.h */

float left_cm = 0.0f;
float front_cm = 100.0f;

uint16_t left_adc = 0;
uint16_t front_adc = 0;

float yaw_deg = 0.0f;
float yaw_rate_dps = 0.0f;

imu_state_t imu;

/* ===================== INTERNAL STATE ===================== */

static uint8_t imu_ready = 0;
static float filtered_left_cm = 0.0f;
static float filtered_front_cm = 0.0f;
static uint8_t is_first_ir_read = 1;

/* ===================== IR CALIBRATION TABLES ===================== */

typedef struct {
    uint16_t d_cm;
    uint16_t adc;
} CalPoint;

/* NOTE: Update these raw ADC values based on your physical testing! */
static const CalPoint cal_left[] = {
 {  7, 745 },
  {  8, 671 },
  { 9, 627},
  { 10, 562 },
  { 12, 490 },
  {13,462},
  {14, 429},
  { 15, 410 },
  {16, 383},
  {17,368},
  { 18, 345 },
  {19, 329},
  { 20, 320 },
  {21, 315},
  { 22, 300 },
  {23, 287},
  {24, 255},
  { 25, 250 },
  {26, 269},
  {27,241},
  { 28, 239},
  {29,238},
  { 30, 233 },
  { 32, 205 },
    { 35, 195 },
  { 38, 185 },
  { 40, 184 }, //
  { 42, 175 },
    { 45, 166 },
  { 50, 154 },
    { 55, 148 },
    { 60, 145 },
  { 65, 129 },
  { 70, 112 },
  { 75, 105 },
  { 80, 96 }
};

static const CalPoint cal_front[] = { //FRONT IS MORE ACCURATE!!
    {  7, 675 },
  {  8, 625 },
  { 9, 610},
  { 10, 533 },
  { 12, 445 },
  {13,420},
  {14, 400},
  { 15, 385 },
  {16, 365},
  {17,340},
  { 18, 330 },
  {19, 315},
  { 20, 298 },
  {21, 290},
  { 22, 280 },
  {23, 265},
  {24, 255},
  { 25, 250 },
  {26, 244},
  {27,241},
  { 28, 232},
  {29,225},
  { 30, 215 },
  { 32, 205 },
    { 35, 195 },
  { 38, 185 },
  { 40, 174 },
  { 42, 166 },
    { 45, 155 },
  { 50, 138 },
    { 55, 129 },
    { 60, 122 },
  { 65, 118 },
  { 70, 112 },
  { 75, 105 },
  { 80, 96 }
//here
  
};

#define CAL_LEFT_N   ((uint8_t)(sizeof(cal_left)  / sizeof(cal_left[0])))
#define CAL_FRONT_N  ((uint8_t)(sizeof(cal_front) / sizeof(cal_front[0])))

/* ===================== ADC HARDWARE FUNCTIONS ===================== */

static void adc_init(void)
{
    /* CRITICAL FIX: Use AVCC (5V) to match Arduino's analogRead calibration */
    ADMUX = (1 << REFS0);

    /* Enable ADC, prescaler 128 -> 125 kHz */
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    ADCSRB = 0;

    /* Disable digital input buffer on ADC0 and ADC1 to save power/reduce noise */
    DIDR0 |= (1 << LEFT_ADC_CHANNEL) | (1 << FRONT_ADC_CHANNEL);
}

static uint16_t adc_read_channel_blocking(uint8_t channel)
{
    /* Preserve upper bits (Voltage Ref), apply channel mask */
    ADMUX = (ADMUX & 0xF0) | (channel & 0x07);
    _delay_us(10); // Wait for multiplexer to settle

    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC)) {}

    return ADC;
}

static uint16_t read_ir_average(uint8_t channel, uint8_t n_samples)
{
    uint32_t sum = 0;
    
    /* Discard first reading after switching channels to clear residual capacitor charge */
    adc_read_channel_blocking(channel); 

    for (uint8_t i = 0; i < n_samples; i++) {
        sum += adc_read_channel_blocking(channel);
        _delay_ms(2); /* CRITICAL: Space out readings to capture real averages */
    }

    return (uint16_t)(sum / n_samples);
}

/* ===================== MATH & INTERPOLATION ===================== */

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

    float d0 = (float)table[i].d_cm;
    float d1 = (float)table[i + 1].d_cm;
    float a0 = (float)table[i].adc;
    float a1 = (float)table[i + 1].adc;
    float den = (a1 - a0);

    if (den == 0.0f) return d0;

    float t = ((float)adc - a0) / den;
    return d0 + t * (d1 - d0);
}

/* ===================== PUBLIC API (Matches sensors.h) ===================== */

uint8_t ir_is_valid(float cm) //Clamping function
{

    if (cm <= IR_MIN_VALID_CM){

        return IR_MIN_VALID_CM;
       
    }else if(cm >= IR_MAX_VALID_CM){

        return IR_MAX_VALID_CM;
    }else{
        return cm;
    }
    
}

void sensors_init(void)
{
    /* 1. Init ADC */
    adc_init();

    /* 2. Init I2C bus */
    I2C_init();
    _delay_ms(100);

    /* 3. Init IMU State */
    imu_ready = 0;
    imu_reset_state(&imu);
    yaw_deg = 0.0f;
    yaw_rate_dps = 0.0f;

    /* 4. Configure MPU6050 */
    if (mpu6050_init() == 0) {
        mpu6050_set_accel_range(AFS_2G);
        mpu6050_set_gyro_range(GFS_500DPS);
        mpu6050_set_dlpf(3);
        mpu6050_set_sample_rate_div(7);

        imu_reset_state(&imu);
        imu_calibrate(&imu, 500);

        imu_ready = 1;
    }

    _delay_ms(100);
}

void update_ir_sensors(void)
{
    /* Read raw ADC values */
    left_adc  = read_ir_average(LEFT_ADC_CHANNEL, IR_SAMPLES);
    front_adc = read_ir_average(FRONT_ADC_CHANNEL, IR_SAMPLES);

    /* Convert to raw centimeters */
    float raw_left  = cm_from_adc_piecewise_extrap(left_adc,  cal_left,  CAL_LEFT_N);
    float raw_front = cm_from_adc_piecewise_extrap(front_adc, cal_front, CAL_FRONT_N);

    /* Apply Adaptive Low-Pass Filter */
    if (is_first_ir_read) {
        filtered_left_cm = raw_left;
        filtered_front_cm = raw_front;
        is_first_ir_read = 0;
    } else {
        /* Filter LEFT sensor */
        float diff_left = fabs(raw_left - filtered_left_cm);
        if (diff_left > 15.0f)      filtered_left_cm = raw_left; // Snap
        else if (diff_left > 5.0f)  filtered_left_cm = (0.50f * filtered_left_cm) + (0.50f * raw_left);
        else                        filtered_left_cm = (0.85f * filtered_left_cm) + (0.15f * raw_left);

        /* Filter FRONT sensor */
        float diff_front = fabs(raw_front - filtered_front_cm);
        if (diff_front > 15.0f)     filtered_front_cm = raw_front; // Snap
        else if (diff_front > 5.0f) filtered_front_cm = (0.50f * filtered_front_cm) + (0.50f * raw_front);
        else                        filtered_front_cm = (0.85f * filtered_front_cm) + (0.15f * raw_front);
    }

    /* Update the public global variables */

    left_cm = ir_is_valid(filtered_left_cm);
    front_cm = ir_is_valid(filtered_front_cm);
    
    
}

void update_imu_sensor(float dt_s)
{
    if (imu_ready && (imu_update(&imu, dt_s) == 0)) {
        yaw_deg = imu.yaw_deg;
        yaw_rate_dps = imu.gz_dps;
    }
}

void update_sensors()
{
    update_ir_sensors();

}