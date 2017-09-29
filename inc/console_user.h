/*
 * console_user.h
 *
 *  Created on: May 2, 2017
 *      Author: Miroslav
 */

#ifndef CONSOLE_USER_H_
#define CONSOLE_USER_H_

#include "definitions.h"

/**
 * If debug console is enabled the function declaration should be compiled
 */
#if defined(DEBUG_CONSOLE) || defined(INFO_CONSOLE)

#include <stm32f4xx.h>
#include <stm32f4xx_usart.h>
#include <stdio.h>

/**
 * Function initialize USART port that will be used as stdout port
 */
error_t console_init();

#endif

#endif /* CONSOLE_USER_H_ */
