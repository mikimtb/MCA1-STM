/*
 * ring_buffer.h
 *
 *  Created on: Apr 6, 2017
 *      Author: Miroslav
 */

#ifndef RING_BUFFER_H_
#define RING_BUFFER_H_

#include <stm32f4xx.h>
#include <stdbool.h>
#include "definitions.h"
/**
 * Type definitions
 */
typedef struct buffer
{
	uint8_t buffer[BUFFER_LENGTH];
	uint8_t next_in;
	uint8_t next_out;
} buffer_t;

/**
 * Function check whether the buffer is empty
 * @param buff[in] pointer to the buffer
 * @return true if buffer is empty, false if buffer is not empty
 */
bool ring_buffer_isEmpty(buffer_t* buff);

/**
 * Function check whether the buffer is full
 * @param buff[in] pointer to the buffer
 * @return true if buffer is full, false if buffer is not full
 */
bool ring_buffer_isFull(buffer_t* buff);

/**
 * Function initialize the buffer
 * @param buff[in] pointer to the buffer that should be initialized
 */
void ring_buffer_init(buffer_t* buff);

/**
 * Function push byte to the ring buffer
 * @param buff[in] pointer to the buffer
 * @param data[in] byte that will be pushed to the buffer
 * @return	false if buffer is full, true if enqueue is successful
 */
bool ring_buffer_enQ(buffer_t* buff, uint8_t data);

/**
 * Function pop one byte from ring buffer
 * @param buff[in] pointer to the buffer
 * @param data[out] data that is dequeued from the buffer
 * @return false if buffer is empty, true if dequeue is successful
 */
bool ring_buffer_deQ(buffer_t* buff, uint8_t* data);

#endif /* RING_BUFFER_H_ */
