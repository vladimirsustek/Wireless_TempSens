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

#define ADC_BUFFER_SIZE 16

#define ADC1_RANK_1 0
#define ADC1_RANK_2 1

const uint32_t VDDA_NOM = 3300;
const uint32_t ADC_MAX = 4095;

/* Calculated from known VDAA comparing the VDAA calc result */
const uint32_t EST_VREFINT = 1158;

static volatile uint32_t adc_buffer[ADC_BUFFER_SIZE];
static volatile uint32_t conv_done = 0;



#ifndef STM32F1
#error VALID ONLY FOR STM32F103C8T6
#endif

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

bool measurement_get(uint32_t* ch, uint32_t* vdda)
{
	if(!conv_done)
	{
		return false;
	}
	else
	{
		conv_done = 0;
	}

	assert(ch != NULL);
	assert(vdda != NULL);

	uint32_t aux_ch = 0, aux_vdda = 0;

	/* Accumulating */
	for(uint32_t idx = 0; idx < ADC_BUFFER_SIZE; idx+=2)
	{
		DEBUG_PRINT("ch[%lu] = %lu\n"
				"vdda[%lu] = %lu\r\n",
				idx + ADC1_RANK_1,
				adc_buffer[idx + ADC1_RANK_1],
				idx + ADC1_RANK_2,
				adc_buffer[idx + ADC1_RANK_2]);
		aux_ch += adc_buffer[idx + ADC1_RANK_1];
		aux_vdda += adc_buffer[idx + ADC1_RANK_2];
	}

	DEBUG_PRINT("\r\naux_ch %lu\n"
			"aux_vdda %lu\r\n",
			aux_ch,
			aux_vdda);

	/* Averaging (when ADC_BUFFER_SIZE is 8)*/
	aux_ch = aux_ch >> 3;
	aux_vdda = aux_vdda >> 3;

	DEBUG_PRINT( "\r\naux_ch >> 3 %lu\n"
			"aux_vdda >> 3 %lu\r\n",
			aux_ch,
			aux_vdda);

	aux_vdda = (ADC_MAX*EST_VREFINT)/(aux_vdda);
	aux_ch = (aux_ch*aux_vdda)/ADC_MAX;

	DEBUG_PRINT( "\r\nvdda %lu\n"
			"ch %lu\r\n",
			aux_vdda,
			aux_ch);

	*ch = aux_ch;
	*vdda = aux_vdda;

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





