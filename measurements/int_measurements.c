#include <assert.h>
#include <int_measurements.h>
#include <string.h>

#include "cli.h"

#include "adc.h"
#include "tim.h"

#include "stm32f1xx.h"

#define ADC_BUFFER_SIZE 6*8

#define OA_SENSE 1
#define NTC_RESISTANCE 1

/* Datasheet stm32f103cb.pdf, page 78 -> internal temperature sensor*/
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

/* Calculated from known VDDA comparing the VDAA calculation result
 * valid only for a unique MCU -> for other needed to be re-defined */
const uint32_t EST_VREFINT = 1158;

static volatile uint32_t adc_buffer[ADC_BUFFER_SIZE];
static volatile uint32_t conv_done = 0;

static int32_t getNTCresistance(uint32_t adc_raw, uint32_t r_ref);
static int32_t lookUpNtcTemperature(uint32_t interp_x);

typedef struct {
	/*  in m°C*/
    int32_t temp;
    /* in Ohms */
    int32_t res;
} TempRes;


/* TDK (EPCOS) B57164K0103K NTC, page 9, variant 2904
 * datasheet docs/dsh.118-010.1.pdf */

#define NTC_10K_LUT_LNG 43
#define NTC_10K_RREF 10000

const TempRes NTC_10K_LUT[NTC_10K_LUT_LNG] = {
		/* m°C             Ohms*/
		{	-55000	,	1214600	},
		{	-50000	,	844390	},
		{	-45000	,	592430	},
		{	-40000	,	419380	},
		{	-35000	,	299470	},
		{	-30000	,	215670	},
		{	-25000	,	156410	},
		{	-20000	,	114660	},
		{	-15000	,	84510	},
		{	-10000	,	62927	},
		{	-5000	,	47077	},
		{	0	,	35563	},
		{	5000	,	27119	},
		{	10000	,	20860	},
		{	15000	,	16204	},
		{	20000	,	12683	},
		{	25000	,	10000	},
		{	30000	,	7942	},
		{	35000	,	6326	},
		{	40000	,	5074	},
		{	45000	,	4102	},
		{	50000	,	3336	},
		{	55000	,	2724	},
		{	60000	,	2237	},
		{	65000	,	1845	},
		{	70000	,	1530	},
		{	75000	,	1275	},
		{	80000	,	1067	},
		{	85000	,	899	},
		{	90000	,	760	},
		{	95000	,	645	},
		{	100000	,	549	},
		{	105000	,	470	},
		{	110000	,	403	},
		{	115000	,	347	},
		{	120000	,	300	},
		{	125000	,	260	},
		{	130000	,	226	},
		{	135000	,	197	},
		{	140000	,	172	},
		{	145000	,	151	},
		{	150000	,	133	},
		{	155000	,	117	},
};

void IntMeas_Open()
{
	/* Start the timer and start a TIM-triggered endless DMA conversion */
	assert(HAL_OK == HAL_TIM_Base_Start(&htim3));
	assert(HAL_OK == HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUFFER_SIZE));
}

void IntMeas_Close()
{
	/* Stop timer and the ADC */
	assert(HAL_OK == HAL_TIM_Base_Stop(&htim3));
	assert(HAL_OK == HAL_ADC_Stop(&hadc1));
}


bool IntMeas_Get(Measurement_t* measurement)
{
	if(!conv_done || measurement == NULL)
	{
		return false;
	}
	else
	{
		conv_done = 0;
	}

	int32_t ntc2 = 0;
	int32_t ntc1 = 0;
	int32_t oagp = 0;
	int32_t curr = 0;
	int32_t tmpi = 0;
	int32_t vdda = 0;

	/* Accumulating */
	for(uint32_t idx = 0; idx < ADC_BUFFER_SIZE; idx+=6)
	{
		ntc2 += adc_buffer[ADC1_RANK_1];
		ntc1 += adc_buffer[ADC1_RANK_2];
		oagp += adc_buffer[ADC1_RANK_3];
		curr += adc_buffer[ADC1_RANK_4];
		tmpi += adc_buffer[ADC1_RANK_5];
		vdda += adc_buffer[ADC1_RANK_6];
	}

	/* Dividing to get an average (2^3) = 8 */
	ntc2 = ntc2 >> 3;
	ntc1 = ntc1 >> 3;
	oagp = oagp >> 3;
	curr = curr >> 3;
	tmpi = tmpi >> 3;
	vdda = vdda >> 3;

	/* Get real VDDA 3V3 */
	vdda = (ADC_MAX*EST_VREFINT)/vdda;

	/* Get voltages of other channels (2^11) = 4096 */
	oagp = (vdda*oagp) >> 12;
	curr = (vdda*curr) >> 12;
	tmpi = (vdda*tmpi) >> 12;

	UART_PRINT("NTC2 %ld\n", (vdda*ntc2) >> 12);
	UART_PRINT("NTC1 %ld\n", (vdda*ntc1) >> 12);

	/* Get interpolated temperature from the NTCs*/
	ntc2 = getNTCresistance(ntc2, NTC_10K_RREF);
#if NTC_RESISTANCE
	UART_PRINT("NTC2 %ld\n", ntc2);
#endif
	ntc2 = lookUpNtcTemperature(ntc2);
	ntc1 = getNTCresistance(ntc1, NTC_10K_RREF);
#if NTC_RESISTANCE
	UART_PRINT("NTC1 %ld\n", ntc1);
#endif
	ntc1 = lookUpNtcTemperature(ntc1);

	/* Get MCU-internal-sensor temperature */
	tmpi = ((V25 - tmpi*1000)/AVG_SLOPE) + 25;

	UART_PRINT(
			"NTC2 %ld\n"
			"NTC1 %ld\n"
#if OA_SENSE
			"OAGP %ld\n"
			"CURR %ld\n"
#endif
			"TMPI %ld\n"
			"VREF %ld\r\n",
			ntc2,
			ntc1,
#if OA_SENSE
			oagp,
			curr,
#endif
			tmpi,
			vdda);

	measurement->ntc2 = ntc2;
	measurement->ntc1 = ntc1;
	measurement->oagp = oagp;
	measurement->curr = curr;
	measurement->tmpi = tmpi;
	measurement->vdda = vdda;

	assert(HAL_OK == HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUFFER_SIZE));

	return true;
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


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

	if(hadc->Instance == ADC1)
	{
			conv_done = 1;
			HAL_ADC_Stop(&hadc1);
	}
}
