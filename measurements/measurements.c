/*
 * measurements.c
 *
 *  Created on: Mar 3, 2023
 *      Author: 42077
 */
#include <assert.h>

#include "cli.h"

#include "measurements.h"

#include "adc.h"
#include "tim.h"

#include "stm32f1xx.h"

#define ADC_BUFFER_SIZE 6*8

/* Datasheet stm32f103cb.pdf, page 78 */
#define AVG_SLOPE (uint32_t)(4300) /* Converted to uV/°C */
#define V25		  (uint32_t)(1430000)    /* Converted to uV */

#define ADC1_RANK_1 0
#define ADC1_RANK_2 1
#define ADC1_RANK_3 2
#define ADC1_RANK_4 3
#define ADC1_RANK_5 4
#define ADC1_RANK_6 5

const uint32_t VDDA_NOM = 3300;
const uint32_t ADC_MAX = 4095;

/* Calculated from known VDAA comparing the VDAA calc result */
const uint32_t EST_VREFINT = 1158;

static volatile uint32_t adc_buffer[ADC_BUFFER_SIZE];
static volatile uint32_t conv_done = 0;

static int32_t getNTCresistance(uint32_t adc_raw, uint32_t r_ref);
static int32_t lookUpNtcTemperature(uint32_t interp_x);

#ifndef STM32F1
#error VALID ONLY FOR STM32F103C8T6
#endif

typedef struct {
    int32_t temp;
    int32_t res;
} TempRes;

#define NTC_10K_LUT_LNG 131

#define NTC_10K_RREF 10000

const TempRes NTC_10K_LUT[NTC_10K_LUT_LNG] = { { -30000, 122840 }, { -29000, 116400 }, { -28000, 110340 },
    { -27000, 104630 }, { -26000, 99250 }, { -25000, 94180 }, { -24000, 89400 }, { -23000, 84890 }, { -22000, 80640 },
    { -21000, 76620 }, { -20000, 72830 }, { -19000, 69250 }, { -18000, 65870 }, { -17000, 62670 }, { -16000, 59600 },
    { -15000, 56780 }, { -14000, 54080 }, { -13000, 51510 }, { -12000, 49090 }, { -11000, 46790 }, { -10000, 44620 },
    { -9000, 42550 }, { -8000, 40600 }, { -7000, 38750 }, { -6000, 36990 }, { -5000, 35320 }, { -4000, 33740 },
    { -3000, 32230 }, { -2000, 30810 }, { -1000, 29450 }, { 0, 28160 }, { 1000, 26920 }, { 2000, 25760 },
    { 3000, 24650 }, { 4000, 23600 }, { 5000, 22600 }, { 6000, 21640 }, { 7000, 20740 }, { 8000, 19870 },
    { 9000, 19050 }, { 10000, 18270 }, { 11000, 17510 }, { 12000, 16800 }, { 13000, 16120 }, { 14000, 15470 },
    { 15000, 14850 }, { 16000, 14260 }, { 17000, 13700 }, { 18000, 13160 }, { 19000, 12640 }, { 20000, 12160 },
    { 21000, 11680 }, { 22000, 11230 }, { 23000, 10800 }, { 24000, 10390 }, { 25000, 10000 }, { 26000, 9624 },
    { 27000, 9265 }, { 28000, 8921 }, { 29000, 8591 }, { 30000, 8276 }, { 31000, 7973 }, { 32000, 7684 },
    { 33000, 7406 }, { 34000, 7140 }, { 35000, 6885 }, { 36000, 6641 }, { 37000, 6406 }, { 38000, 6181 },
    { 39000, 5965 }, { 40000, 5758 }, { 41000, 5559 }, { 42000, 5368 }, { 43000, 5185 }, { 44000, 5008 },
    { 45000, 4839 }, { 46000, 4676 }, { 47000, 4520 }, { 48000, 4370 }, { 49000, 4225 }, { 50000, 4086 },
    { 51000, 3953 }, { 52000, 3824 }, { 53000, 3700 }, { 54000, 3581 }, { 55000, 3467 }, { 56000, 3356 },
    { 57000, 3250 }, { 58000, 3147 }, { 59000, 3049 }, { 60000, 2954 }, { 61000, 2862 }, { 62000, 2774 },
    { 63000, 2689 }, { 64000, 2607 }, { 65000, 2528 }, { 66000, 2451 }, { 67000, 2378 }, { 68000, 2306 },
    { 69000, 2238 }, { 70000, 2172 }, { 71000, 2108 }, { 72000, 2046 }, { 73000, 1986 }, { 74000, 1929 },
    { 75000, 1873 }, { 76000, 1820 }, { 77000, 1768 }, { 78000, 1717 }, { 79000, 1669 }, { 80000, 1622 },
    { 81000, 1577 }, { 82000, 1533 }, { 83000, 1490 }, { 84000, 1449 }, { 85000, 1410 }, { 86000, 1371 },
    { 87000, 1334 }, { 88000, 1298 }, { 89000, 1263 }, { 90000, 1229 }, { 91000, 1197 }, { 92000, 1165 },
    { 93000, 1134 }, { 94000, 1105 }, { 95000, 1076 }, { 96000, 1048 }, { 97000, 1021 }, { 98000, 995 }, { 99000, 969 },
    { 100000, 945 } };

void measurements_open()
{
	/* Start the timer and start a TIM-triggered endless DMA conversion */
	assert(HAL_OK == HAL_TIM_Base_Start(&htim3));
	assert(HAL_OK == HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUFFER_SIZE));
}

void measurements_close()
{
	/* Stop timer and the ADC */
	assert(HAL_OK == HAL_TIM_Base_Stop(&htim3));
	assert(HAL_OK == HAL_ADC_Stop(&hadc1));
}

bool measurement_get()
{
	if(!conv_done)
	{
		return false;
	}
	else
	{
		conv_done = 0;
	}

	int32_t ntc2 = 0;
	int32_t ntc1 = 0;
	uint32_t oagp = 0;
	uint32_t curr = 0;
	uint32_t temp = 0;
	uint32_t vdda = 0;

	/* Accumulating */
	for(uint32_t idx = 0; idx < ADC_BUFFER_SIZE; idx+=6)
	{
		ntc2 += adc_buffer[ADC1_RANK_1];
		ntc1 += adc_buffer[ADC1_RANK_2];
		oagp += adc_buffer[ADC1_RANK_3];
		curr += adc_buffer[ADC1_RANK_4];
		temp += adc_buffer[ADC1_RANK_5];
		vdda += adc_buffer[ADC1_RANK_6];
	}

	/* Dividing to get an average (2^3) = 8 */
	ntc2 = ntc2 >> 3;
	ntc1 = ntc1 >> 3;
	oagp = oagp >> 3;
	curr = curr >> 3;
	temp = temp >> 3;
	vdda = vdda >> 3;

	/* Get real VDDA 3V3 */
	vdda = (ADC_MAX*EST_VREFINT)/vdda;

	/* Get voltages of other channels (2^11) = 4096 */
	oagp = (vdda*oagp) >> 12;
	curr = (vdda*curr) >> 12;
	temp = (vdda*temp) >> 12;

	/* Get interpolated temperature from the NTCs*/
	ntc2 = getNTCresistance(ntc2, NTC_10K_RREF);
	ntc2 = lookUpNtcTemperature(ntc2);
	ntc1 = getNTCresistance(ntc1, NTC_10K_RREF);
	ntc1 = lookUpNtcTemperature(ntc1);

	/* Get MCU-internal-sensor temperature */
	temp = ((V25 - temp*1000)/AVG_SLOPE) + 25;

	DEBUG_PRINT(
			"-------\n"
			"NTC2 %ld\n"
			"NTC1 %ld\n"
			"OAGP %ld\n"
			"CURR %ld\n"
			"TMPI %ld\n"
			"VREF %ld\r\n",
			ntc2,
			ntc1,
			oagp,
			curr,
			temp,
			vdda);

	assert(HAL_OK == HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUFFER_SIZE));

	return true;
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

	if(hadc->Instance == ADC1)
	{
			conv_done = 1;
			HAL_ADC_Stop(&hadc1);
	}
}

static int32_t getNTCresistance(uint32_t adc_raw, uint32_t r_ref)
{
	/*  --o----- V_DDA
	 *    |
	 *  |---|
	 *  |   |  R_REF
	 *  |   |
	 *  |---|
	 *    |
	 *    o-----V_RNTC ... sensed by an ADC
	 *    |
	 *  |---|
	 *  |   |  R_NTC
	 *  |   |
	 *  |---|
	 *    |
	 *   GND
	 *
	 * For a voltage divider of the R_NTC and
	 * the R_REF Pull-up connected to the V_DDA:
	 *
	 * V_RTNC = R_NTC/(R_NTC+R_REF)*V_DDA
	 *
	 * And a typical ADC voltage expression:
	 *
	 * V_RNTC = ADC_RAW*V_DDA/ADC_RESOLUTION
	 *
	 * Setting these two equations equal:
	 *
	 * R_NTC/(R_NTC+R_REF) * V_DDA = ADC_RAW * V_DDA/ADC_RESOLUTION ... eliminated V_DDA
	 *
	 * R_TNC/(R_NTC+R_REF) = RAW/ADC_RESOLUTION
	 *
	 * R_NTC = ADC_RAW/ADC_RESOLUTION * (R_NTC+RREF)
	 *
	 * R_NTC = ADC_RAW/ADC_RESOLUTION * R_NTC + ADC_RAW/ADC_RESOLUTION * R_REF
	 *
	 * R_NTC - ADC_RAW/ADC_RESOLUTION * R_NTC = ADC_RAW/ADC_RESOLUTION * R_REF
	 *
	 * R_NTC * (1 - ADC_RAW/ADC_RESOLUTON) = ADC_RAW/ADC_RESOLUTION *R_REF
	 *
	 * R_NTC = ADC_RAW/ADC_RESOLUTION / (1 - ADC_RAW/ADC_RESOLUTION)
	 *
	 * To avoid loosing precision when dividing ADC_RAW/ADC_RESOLUTION
	 * equal is multiplied 1024 (shift left 10)
	 *
	 * SUBFRACTION = ADC_RAW/ADC_RESOLUTION * 1024
	 *
	 * R_NTC = R_REF * SUBFRACTION / (1024 - SUBFRACTION)
	 *
	 *
	 * */

    int64_t subfraction = (((int64_t)(adc_raw) << 10) >> 12);

    int64_t temp = (int64_t)(r_ref) * subfraction;

    return (int32_t)(temp / (1024 - subfraction));
}

static int32_t lookUpNtcTemperature(uint32_t interp_x)
{
    for (uint32_t idx = 0; idx < NTC_10K_LUT_LNG - 1; idx++) {
        if (NTC_10K_LUT[idx].res >= (int32_t)(interp_x) &&
            NTC_10K_LUT[idx + 1].res <= (int32_t)(interp_x)) {

            /* Linear interpolation y = y0 + (x - x0)*(y1-y0)/(x1-x0) */

            /* (x - x0) */
            int64_t temp = ((int32_t)interp_x - NTC_10K_LUT[idx].res);

            /* multiply 'temp' by the (y1 - y0) */
            temp *= (NTC_10K_LUT[idx + 1].temp - NTC_10K_LUT[idx].temp);

            /* divide 'temp' by the (x1 - x0) */
            temp /= (NTC_10K_LUT[idx + 1].res - NTC_10K_LUT[idx].res);

            /* add y0 + 'temp' */
            return (int32_t)(NTC_10K_LUT[idx].temp + temp);
        }
    }
    return INT32_MIN;
}
