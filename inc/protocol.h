/**
 * protocol.h
 *
 *  Created on: Apr 9, 2017
 *      Author: Miroslav
 *
 * Library for serial communication using custom protocol
 * Data package structure:
 *   # [DEV_ADR] [MESSAGE_ID] [LENGTH] [DATA<0>,...DATA<LENGTH-1>] [CRC] $
 *
 */

#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#include <stm32f4xx.h>
#include "uart_user.h"
#include "definitions.h"

/**
 * Constant definitions
 */
#define DATA_MAX_LENGTH		32					/** Maximal length of data array in packet */
#define GP  				0x107   			/** CRC8 polynom x^8 + x^2 + x + 1 */
#define DI  				0x07
#define PACKET_HEADER_SIZE 	3					/** Define packet header size in number of bytes that are used */
#define CRC_SIZE			1					/** Define how many bytes are used for CRC */

/**
 * Parser state machine states definition
 */
typedef enum
{
	PARSE_ADDRESS,
	PARSE_ID,
	PARSE_LENGTH,
	RECEIVE_DATA,
	PARSE_CRC8,
	SEND_NACK,
	PROCESS_MSG,
	N_STATES
} state_e;

typedef enum
{
	reset_sm,
	adr_received,
	ID_received,
	length_received,
	parse_data,
	data_received,
	CRC8_failed,
	CRC8_ok,
	N_TRANSITIONS
} transition_e;

/**
 * structure that is used to handle transition between FSM states
 */
struct state_transition
{
	state_e (*state_fptr)(transition_e *e);
};

/*
 * User type definitions
 */
/**
 * Packet type definition
 */
typedef struct __attribute__((packed))
{
	uint8_t DEV_ADR;
	uint8_t ID;
	uint8_t LENGTH;
	uint8_t DATA[DATA_MAX_LENGTH];
} packet_t;

/**
 * Function definition
 */
/**
 * Function that will initialize protocol state machine
 */
void protocol_init_state_machine(void);
/**
 * Function call next callback in parser state machine
 * @return true if complete message is received, false if message is still uncompleted
 */
bool protocol_parse_uart_data(void);
/**
 * Function return device address
 * @return address that is used on a bus to identify the device
 */
uint8_t protocol_get_devAdr();
/**
 * Function return message ID of received message
 * @return message ID of last received message
 */
uint8_t protocol_get_msgID();
/**
 * Function return message data length
 * @return DATA length of last received message
 */
uint8_t protocol_getLENGTH();
/**
 * The function return pointer to the array of DATA inside receive packet
 * @return pointer to the array with DATA
 */
uint8_t* protocol_getDATA();
/**
 * Function send the message over serial network
 * @param msg[in]	pointer to the message
 * @param msg_size  message size
 * @return true if message has been sent, false if message size is out of range
 */
bool protocol_send_message(uint8_t* msg, uint16_t msg_size, uint8_t response_id);

#endif /* PROTOCOL_H_ */
