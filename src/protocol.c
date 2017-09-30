/*
 * protocol.c
 *
 *  Created on: Apr 9, 2017
 *      Author: Miroslav
 */
#include "protocol.h"
#include <stdbool.h>
#include <string.h>

/**
 * Static function prototypes
 */
static void wait_for_adr();
static void parse_id();
static void parse_length();
static void parse_data();
static void parse_crc();

/**
 * Global variables
 */
/** Callback function pointer, initialized to point on wait for start at the beginning */
fptr_t parse_next = wait_for_adr;
/** Data structure that will be used to parse received data into message */
packet_t packet_in = {0, 0, 0, {0}};
packet_t packet_out = {0, 0, 0, {0}};
/** FLAG that trigger message parser */
static bool NEW_MSG_RECEIVED_FLAG = false;
/** Counter that is used to count number of parsed data */
static uint16_t rx_count;
/** CRC8 8bit lookup table */
static unsigned char crc8_table[256];
/** Flag that trigger initialization of crc8_table after first packet is received or transmitted */
static bool made_table=0;

/**
 * Private functions
 */

/**
 * Function initialize crc8 lookup table
 * it should be called before any other function that is calling crc8 function
 */
static void init_crc8()
{
  int i,j;
  uint8_t crc;

  if (!made_table)
  {
    for (i=0; i<256; i++)
    {
      crc = i;
      for (j=0; j<8; j++)
        crc = (crc << 1) ^ ((crc & 0x80) ? DI : 0);
      crc8_table[i] = crc & 0xFF;
    }
    made_table = true;
  }
}

/**
 * Function calculate crc8 for given packet
 * @param p[in] pointer to the packet type
 * @param size[in] the size of packet
 * @return resulting crc8
 */
static uint8_t crc8(packet_t *p, uint16_t size)
{
	uint8_t crc = 0;
	/** If the crc8 lookup table isn't made yet, init_crc8() will be called before calculation */
	if (!made_table)
	{
	    init_crc8();
	}
	/** Calculate crc8 */
	for (int i=0; i<size; i++)
	{
		crc = crc8_table[crc ^ ((uint8_t*)p)[i]];
		crc &= 0xff;
	}

	return crc;
}

/**
 * Callback that wait for device address character
 */
static void wait_for_adr()
{
    /** If there is no data available, return */
    if (!uart_bkbhit())
        return;

    uint8_t c;

    /** Check for the data */
    if (uart_bgetc(&c))
    {
    	/** Store device ADDRESS to the packet buffer */
    	packet_in.DEV_ADR = c;
    	rx_count++;
    	/** Go to next state */
    	parse_next = parse_id;
    }
}

/**
 * Callback that read third character identifier, stores it in data_id
 */
static void parse_id()
{
    /** If there is no data available, return */
    if (!uart_bkbhit())
        return;

    uint8_t c;

    /** Check for the data */
    if (uart_bgetc(&c))
    {
    	/** Store device ADDRESS to the packet buffer */
    	packet_in.ID = c;
    	rx_count++;
    	/** Go to next state */
    	parse_next = parse_length;
    }
}

/**
 * Callback that read protocol data length
 */
static void parse_length()
{
    /** If there is no data available, return */
    if (!uart_bkbhit())
        return;

    uint8_t c;

    /** Check for the data */
    if (uart_bgetc(&c))
    {
    	/** Store device ADDRESS to the packet buffer */
    	packet_in.LENGTH = c;
    	rx_count++;
    	/** Go to next state */
    	parse_next = parse_data;
    }
}

/**
 * Callback that read data and store it in packet.DATA buffer
 */
static void parse_data()
{
    /** If there is no data available, return */
    if (!uart_bkbhit())
        return;

    uint8_t c;
    /**
     * ADR, ID and LENGTH are received. Next is payload which should be stored in
     * DATA buffer. This state will be active until LENGTH bytes are not received
     */
    /** Check for the data */
	if (uart_bgetc(&c))
	{
		packet_in.DATA[rx_count - PACKET_HEADER_SIZE] = c;
		rx_count++;
		/** State transition rule check */
		if ((rx_count-PACKET_HEADER_SIZE) == packet_in.LENGTH)
		{
			parse_next = parse_crc;			/** Go to next state */
		}
	}
}

/**
 * Callback that read checksum and store it in packet.CRC8
 */
static void parse_crc()
{
	/** If there is no data available, return */
	if (!uart_bkbhit())
		return;

	uint8_t c;

	/** Check for the data */
	if (uart_bgetc(&c))
	{
		packet_in.DATA[rx_count - PACKET_HEADER_SIZE] = c;
		rx_count++;
		/** Check CRC */
		if (packet_in.DATA[rx_count - PACKET_HEADER_SIZE - 1] != crc8(&packet_in, rx_count-1))
		{
			rx_count = 0;
			parse_next = wait_for_adr;
			/**
			 *  Request for retransmission should be added here if it's needed
			 *  printf(uart_bputc, "CRC failed!\r\n");
			 */
			return;
		}

		NEW_MSG_RECEIVED_FLAG = true;

		rx_count = 0;
		parse_next = wait_for_adr;
	}
}

/**
 * Public functions
 */

bool protocol_parse_uart_data()
{
	bool msg_rcv_done = false;
    /** check state machine */
	while(uart_bkbhit() && (!NEW_MSG_RECEIVED_FLAG))
	{
		(*parse_next)();
	}

	if (NEW_MSG_RECEIVED_FLAG)
	{
		msg_rcv_done = true;
		NEW_MSG_RECEIVED_FLAG = false;
	}

	return msg_rcv_done;
}

uint8_t protocol_get_devAdr()
{
	return packet_in.DEV_ADR;
}

uint8_t protocol_get_msgID()
{
	return packet_in.ID;
}

uint8_t protocol_getLENGTH()
{
	return packet_in.LENGTH;
}

uint8_t* protocol_getDATA()
{
	return packet_in.DATA;
}

bool protocol_send_message(uint8_t* msg, uint16_t msg_size, uint8_t response_id)
{
	if(!(msg_size < DATA_MAX_LENGTH))
	{
		return false;
	}

	packet_out.DEV_ADR = DEVICE_NETWORK_ADDRESS;
	packet_out.ID = response_id;
	packet_out.LENGTH = msg_size;
	memcpy(packet_out.DATA, msg, msg_size);
	packet_out.DATA[msg_size] = crc8(&packet_out, (msg_size + PACKET_HEADER_SIZE));

	//uint8_t temp = ((uint8_t*)p)[i];
	uart_write(&packet_out, (PACKET_HEADER_SIZE + msg_size + CRC_SIZE));

	return true;

}
