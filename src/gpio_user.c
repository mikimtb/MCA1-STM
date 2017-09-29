/*
 * gpio_user.c
 *
 *  Created on: Apr 6, 2017
 *      Author: Miroslav
 */
#include "gpio_user.h"

void gpio_init()
{
	GPIO_InitTypeDef GPIO_InitDef;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_InitDef.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitDef.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitDef.GPIO_OType = GPIO_OType_PP;
	GPIO_InitDef.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitDef.GPIO_Speed = GPIO_Speed_50MHz;

	/** Initialize pins */
	GPIO_Init(GPIOC, &GPIO_InitDef);
}
