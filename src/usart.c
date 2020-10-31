/*
 * usart.c
 *
 *  Created on: 28 окт. 2020 г.
 *      Author: Tugrik
 */

#include "usart.h"

uint8_t data_rx;
uint8_t data_tx = 0;
uint8_t buffer_tx[BUFFER_TX_SIZE];
uint8_t buffer_rx[BUFFER_RX_SIZE];

void buffer_init() {
	for (int i = 0; i < BUFFER_TX_SIZE; i++) {
		buffer_tx[i] = i;
	}
}


void DMA1_Channel2_3_IRQHandler() {
	if (DMA1->ISR & DMA_ISR_TCIF2) {
		DMA1->IFCR |= DMA_IFCR_CTCIF2;
	}

	if (DMA1->ISR & DMA_ISR_TCIF3) {
		DMA1->IFCR |= DMA_IFCR_CTCIF3;
		usart1_send_via_dma();
		usart1_get_via_dma();
	}
}


void USART1_IRQHandler() {
	if (USART1->ISR & USART_ISR_TC) {
		USART1->ICR |= USART_ICR_TCCF;
		usart1_tx();
	}

	if (USART1->ISR & USART_ISR_RXNE) {
		data_rx = USART1->RDR;
	}
}


//static void usart2_init_gpio() {
//	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
//	GPIOA->MODER |= GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1;
//	GPIOA->AFR[0] |= (1 << 8)| (1 << 12);
//}

static void usart1_init_gpio() {
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA->MODER |= GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1;
	GPIOA->AFR[1] |= (1 << 4)| (1 << 8);
}

static void usart1_dma_init_tx() {

	buffer_init();
	dbg_init_gpio();

	RCC->AHBENR|= RCC_AHBENR_DMA1EN;

	DMA1_Channel2->CNDTR = BUFFER_TX_SIZE;
	DMA1_Channel2->CMAR = (uint32_t)(&buffer_tx[0]);
	DMA1_Channel2->CPAR = (uint32_t)(&(USART1->TDR));
	DMA1_Channel2->CCR |= DMA_CCR_MINC | DMA_CCR_DIR;

	DMA1_Channel2->CCR |= DMA_CCR_TCIE;

	SYSCFG->CFGR1 |= SYSCFG_CFGR1_USART1TX_DMA_RMP;

	NVIC_SetPriority(DMA1_Channel2_3_IRQn, 8);
	NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

	USART1->CR3 |= USART_CR3_DMAT;

	DMA1_Channel2->CCR |= DMA_CCR_EN;
}

static void usart1_send_via_dma() {
	DMA1_Channel2->CCR &= ~DMA_CCR_EN;
	DMA1_Channel2->CNDTR = BUFFER_RX_SIZE;
	DMA1_Channel2->CMAR = (uint32_t)(&buffer_rx[0]);
	DMA1_Channel2->CCR |= DMA_CCR_EN;
}

static void usart1_get_via_dma() {
	DMA1_Channel3->CCR &= ~DMA_CCR_EN;
	DMA1_Channel3->CNDTR = BUFFER_RX_SIZE;
	DMA1_Channel3->CCR |= DMA_CCR_EN;
}

static void usart1_dma_init_rx() {
	RCC->AHBENR|= RCC_AHBENR_DMA1EN;

	DMA1_Channel3->CNDTR = BUFFER_RX_SIZE;
	DMA1_Channel3->CMAR = (uint32_t)(&buffer_rx[0]);
	DMA1_Channel3->CPAR = (uint32_t)(&(USART1->RDR));
	DMA1_Channel3->CCR |= DMA_CCR_MINC;

	DMA1_Channel3->CCR |= DMA_CCR_TCIE;

	SYSCFG->CFGR1 |= SYSCFG_CFGR1_USART1RX_DMA_RMP;

	NVIC_SetPriority(DMA1_Channel2_3_IRQn, 8);
	NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

	USART1->CR3 |= USART_CR3_DMAR;

	DMA1_Channel3->CCR |= DMA_CCR_EN;
}

void usart1_tx() {
	USART1->TDR = data_tx++;
	if (data_tx % 255 == 0) {
		dbg_trig_gpio();
	}
}

void usart1_init() {
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	usart1_init_gpio();

	USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;	//enable transmit and receive
	USART1->CR3 |= USART_CR3_OVRDIS;	//only for debug!!!

	USART1->BRR = 8000000/115200;

//	USART1->CR1 |= USART_CR1_RXNEIE | USART_CR1_TCIE;	// | USART_CR1_TXEIE;	//interruption
//	NVIC_SetPriority(USART1_IRQn, 3);
//	NVIC_EnableIRQ(USART1_IRQn);

	usart1_dma_init_tx();
	usart1_dma_init_rx();

	USART1->CR1|= USART_CR1_UE;
}
