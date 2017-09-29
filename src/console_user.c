/*
 * console_user.c
 *
 *  Created on: May 2, 2017
 *      Author: Miroslav
 */
#include "console_user.h"

#if defined(DEBUG_CONSOLE) || defined(INFO_CONSOLE)

int __io_putchar(int ch)
{
	// wait until data register is empty
	while( !(USART1->SR & 0x00000040) );
	USART_SendData(USART1, ch);

	return 0;
}

error_t console_init()
{
	GPIO_InitTypeDef GPIO_InitStruct; 		/** this is for the GPIO pins used as TX and RX */
	USART_InitTypeDef USART_InitStruct; 	/** this is for the USART4	 initialization */
	//NVIC_InitTypeDef NVIC_InitStructure; 	/** this is used to configure the NVIC (nested vector interrupt controller) */

	/**
	 * Enable the peripheral clock for the pins used by
	 * USART1, PA0 for TX and PA1 for RX
	 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/**
	 * The RX and TX pins are now connected to their AF
	 * so that the USART4 can take over control of the pins
	 */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

	/**
	 * This sequence sets up the TX and RX pins
	 * so they work correctly with the USART4 peripheral
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; /** Pins 0 (TX) and 1 (RX) are used */
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 			/** the pins are configured as alternate function so the USART peripheral has access to them */
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		/** this defines the IO speed and has nothing to do with the baudrate */
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			/** this defines the output type as push pull mode (as opposed to open drain) */
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			/** this activates the pull up resistors on the IO pins */
	GPIO_Init(GPIOA, &GPIO_InitStruct);					/** now all the values are passed to the GPIO_Init() function which sets the GPIO registers */

	/**
	 * Enable APB1 peripheral clock for USART4
	 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	/**
	 * Now the USART_InitStruct is used to define the properties of USART4
	 */
	USART_InitStruct.USART_BaudRate = 115200;										/** the baudrate is set to the value we passed into this init function */
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;						/** we want the data frame size to be 8 bits (standard) */
	USART_InitStruct.USART_StopBits = USART_StopBits_1;								/** we want 1 stop bit (standard) */
	USART_InitStruct.USART_Parity = USART_Parity_No;								/** we don't want a parity bit (standard) */
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; 	/** we don't want flow control (standard) */
	USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; 					/** we want to enable the transmitter and the receiver */
	USART_Init(USART1, &USART_InitStruct);											/** again all the properties are passed to the USART_Init
																					 *  function which takes care of all the bit setting */
	/**
	 * Here the USART4 receive interrupt is enabled
	 * and the interrupt controller is configured
	 * to jump to the USART2_IRQHandler() function
	 * if the USART2 receive interrupt occurs
	 */
//	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;		 						/** we want to configure the USART2 interrupts */
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;						/** this sets the priority group of the USART2 interrupts */
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 						/** this sets the subpriority inside the group */
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 						/** the USART2 interrupts are globally enabled */
//	NVIC_Init(&NVIC_InitStructure);							 						/** the properties are passed to the NVIC_Init */
//
//	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); 									/** enable the USART2 receive interrupt */

	// finally this enables the complete UART4 peripheral
	USART_Cmd(USART1, ENABLE);

	return no_error;
}

#endif

