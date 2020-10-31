/*
 * adc.c
 *
 *  Created on: 17 окт. 2020 г.
 *      Author: Tugrik
 */

#include "adc.h"

uint16_t adc_data_buf[ADC_DATA_DMA];

uint16_t adc_data;
float real_volt;
void ADC1_COMP_IRQHandler(void) {
	adc_data = ADC1->DR;	//adc_get_data();
	real_volt = adc_data * 0.00081;
	ADC1->ISR |= ADC_ISR_EOC;	//reset interruption!!!

}

void DMA1_Channel1_IRQHandler(void) {
	DMA1->IFCR |= DMA_IFCR_CTCIF1;

	DMA1_Channel1->CCR &= ~DMA_CCR_EN;
	DMA1_Channel1->CNDTR = 1024;
	DMA1_Channel1->CCR |= DMA_CCR_EN;
	adc_start_conversion();
}

//TO-DO this function for init of adc channels
static void adc_init_gpio(uint16_t channels_adc) {
	//adc_in11 PC1
	if (channels_adc & ADC_IN11) {
		RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
		GPIOC->MODER |= GPIO_MODER_MODER1;	//PC1
	}
}

extern uint32_t SystemCoreClock;
void tim2_count_ms(uint16_t msec, uint8_t start_timer) {
	SystemCoreClock;	//F_osc
	TIM2->CR1 &= ~TIM_CR1_CEN;

	//выполнить расчёт задержки msec
	TIM2->ARR = 8000 - 1;	//FIXED param
	TIM2->PSC = 100 - 1;	//var param

	if (start_timer == 1) {
		TIM2->CR1 |= TIM_CR1_CEN;
	}
}

/*
 *  it can be only called in function adc_init()
 */
static void adc_EnableTRG0_tim2(uint16_t time_conv) {
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	tim2_count_ms(time_conv, STOP_TIMER);
//------------------------------------
	//need generate trg0!!!
	TIM2->CR2 |= TIM_CR2_MMS_1;	//generate TRG0 after overflow TIM2_CNT

	TIM2->CR1 |= TIM_CR1_CEN;

//	set EXTEN = 01 -- it's triggered by rising pulse
	ADC1->CFGR1 |= ADC_CFGR1_EXTEN_0;
	ADC1->CFGR1 |= ADC_CFGR1_EXTSEL_1;

	ADC1->CFGR1 &= ~ADC_CFGR1_CONT;
	ADC1->CFGR1 |= ADC_CFGR1_DISCEN;	//continuous conversion mode hardware or software
}

void ADC_DMA_init() {
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	DMA1_Channel1->CCR |= DMA_CCR_MSIZE_0;	//16 bit
	DMA1_Channel1->CCR |= DMA_CCR_PSIZE_0;	//16 bit
	DMA1_Channel1->CCR |= DMA_CCR_MINC;
	DMA1_Channel1->CCR |= DMA_CCR_TCIE;	//set interrupt when complete dma work

	DMA1_Channel1->CNDTR = ADC_DATA_DMA;
	DMA1_Channel1->CPAR = (uint32_t)(&(ADC1->DR));	//??????
	DMA1_Channel1->CMAR = (uint32_t)&adc_data_buf[0];

	SYSCFG->CFGR1 |= SYSCFG_CFGR1_ADC_DMA_RMP; 	//1 << 8;

	NVIC_SetPriority(DMA1_Channel1_IRQn, 5);
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);

	DMA1_Channel1->CCR |= DMA_CCR_EN;	//enable
	ADC1->CFGR1 |= ADC_CFGR1_DMAEN;
}

void adc_init(uint8_t control, uint16_t time_conversion, uint16_t channel_adc) {
	adc_init_gpio(channel_adc);

	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

	RCC->CR2 |= RCC_CR2_HSI14ON;	//turn on hsi14
	while(!(RCC->CR2 && RCC_CR2_HSI14RDY));

	ADC1->SMPR |= ADC_SMPR_SMP;	//239.5 adc clock cycles
	ADC1->CHSELR |= ADC_CHSELR_CHSEL11;	//pc1

	ADC1->CFGR1 |= ADC_CFGR1_CONT;	//continuous conversion mode	ONLY!!! for software trigger
	if (control & ADC_SINGLE_MEAS) {
		ADC1->CFGR1 &= ~ADC_CFGR1_CONT;
	}

	if (control & ADC_USE_TRG0) {
		adc_EnableTRG0_tim2(time_conversion);	//every xxx msec start ad conversion
	}

	if (control & ADC_USE_DMA) {
		ADC_DMA_init();
	}
	else {
		ADC1->IER |= ADC_IER_EOCIE;	//init interruption!!!
		NVIC_SetPriority(ADC1_COMP_IRQn, 8);
		NVIC_EnableIRQ(ADC1_COMP_IRQn);
	}

	ADC1->CR |= ADC_CR_ADEN;	//turn on adc...

	adc_start_conversion();
}

void adc_start_conversion() {
	ADC1->CR |= ADC_CR_ADSTART;
}

uint16_t adc_get_data() {
	return ADC1->DR;
}
