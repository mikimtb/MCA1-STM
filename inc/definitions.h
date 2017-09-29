/*
 * definitions.h
 *
 *  Created on: Apr 2, 2017
 *      Author: Miroslav
 */

#ifndef DEFINITIONS_H_
#define DEFINITIONS_H_

/**
 * Error handling defines
 */
typedef enum
{
	no_error = 0,
	error = -1
} error_t;

/**
 * Console enable
 */
#define DEBUG_CONSOLE 1
#define INFO_CONSOLE 1

/**
 * Communication parameters
 */
#define DEVICE_NETWORK_ADDRESS 0x01

/**
 * System buffer sizes
 */
#define BUFFER_LENGTH 	64

#endif /* DEFINITIONS_H_ */
