/*
 * dbg.c
 *
 *  Created on: 31 окт. 2020 г.
 *      Author: еу
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
