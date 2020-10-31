/*
 * adc.h
 *
 *  Created on: 17 окт. 2020 г.
 *      Author: Tugrik
 */

#ifndef ADC_H_
#define ADC_H_

#include <stm32f0xx.h>
#include "dbg.h"


#define STOP_TIMER	0
#define START_TIMER	1
#define NO_TIME_DELAY 0

#define ADC_NOT_SPEC_SETT	0
#define	ADC_USE_TRG0	(1 << 0)
#define ADC_USE_DMA		(1 << 1)
#define	ADC_SINGLE_MEAS	(1 << 2)
#define ADC_DBG	(1 << 3)

#define	ADC_IN0		0	//GPIOA
#define	ADC_IN8		2	//GPIOB
#define	ADC_IN11	1	//GPIOC


#define ADC_DATA_DMA	1024

/*
 * control settings of adc
 */
void adc_init(uint8_t control, uint16_t per_conversion, uint16_t channel_adc);
void adc_start_conversion();
uint16_t adc_get_data();



#endif /* ADC_H_ */
