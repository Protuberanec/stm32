/*
 * dbg.c
 *
 *  Created on: 31 îêò. 2020 ã.
 *      Author: åó
 */
#include "dbg.h"

/*
 * blink led...
 */
void dbg_init_gpio() {
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	GPIOC->MODER |= GPIO_MODER_MODER8_0;
}

//1 xor 1 = 0
//0 xor 0 = 0
//1 xor 0 = 1
//0 xor 1 = 1

void dbg_trig_gpio() {
	GPIOC->ODR ^= GPIO_ODR_8;
}

//test make comments....!!!
