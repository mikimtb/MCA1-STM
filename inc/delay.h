/*
 * delay.h
 *
 *  Created on: Apr 30, 2017
 *      Author: Miroslav
 */

#ifndef DELAY_H_
#define DELAY_H_

#include <stm32f4xx.h>

/**
 * Function initialize delay timer engine
 */
void delay_init();

/**
 * Function produce delay of <value> milliseconds
 * @param value[in] the time in milliseconds that delay function should wait for
 */
void delay_ms(volatile uint32_t value);

#endif /* DELAY_H_ */
