/*
 * usart.h
 *
 *  Created on: 28 окт. 2020 г.
 *      Author: Tugrik
 */

#ifndef USART_H_
#define USART_H_

#include <stm32f0xx.h>
#include "dbg.h"

#define	BUFFER_TX_SIZE	256
#define	BUFFER_RX_SIZE	16

void usart1_init();

static void usart1_send_via_dma();
static void usart1_get_via_dma();
void usart1_tx();

#endif /* USART_H_ */
