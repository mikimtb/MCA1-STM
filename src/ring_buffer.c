/*
 * ring_buffer.c
 *
 *  Created on: Apr 6, 2017
 *      Author: Miroslav
 */
#include "ring_buffer.h"
/**
 * Private functions
 */

/**
 * Function return how many bytes are stored in the buffer
 * @param buff[in] pointer to the buffer
 * @return number of stored bytes
 */
static uint32_t get_count(buffer_t* buff)
{
	return (buff->next_in - buff->next_out);
}

/**
 * Public functions
 */

bool ring_buffer_isEmpty(buffer_t* buff)
{
	return (get_count(buff) == 0);
}

bool ring_buffer_isFull(buffer_t* buff)
{
	return (get_count(buff) == BUFFER_LENGTH);
}

void ring_buffer_init(buffer_t* buff)
{
	buff->next_in = 0;
	buff->next_out = 0;
}

bool ring_buffer_enQ(buffer_t* buff, uint8_t data)
{
	bool result = !ring_buffer_isFull(buff);
	if (result)
	{
		buff->buffer[buff->next_in++ % BUFFER_LENGTH] = data;
	}

	return result;
}

bool ring_buffer_deQ(buffer_t* buff, uint8_t* data)
{
	bool result = !ring_buffer_isEmpty(buff);
	if (result)
	{
		*data = buff->buffer[buff->next_out++ % BUFFER_LENGTH];
	}

	return result;
}
