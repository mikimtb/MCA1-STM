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
static state_e wait_for_adr(transition_e *e);
static state_e wait_id(transition_e *e);
static state_e wait_length(transition_e *e);
static state_e wait_data(transition_e *e);
static state_e wait_crc(transition_e *e);
static state_e send_NACK(transition_e *e);
static state_e process_msg(transition_e *e);
static bool sm_apply_event(state_e *s, transition_e *e);

/**
 * Protocol parser state machine transition matrix
 */
struct state_transition state_trans[N_TRANSITIONS][N_STATES] =
{
/*						 PARSE_ADDRESS   PARSE_ID   PARSE_LENGTH   RECEIVE_DATA  PARSE_CRC8     SEND_NACK    PROCESS_MSG */
/* reset_sm */			{wait_for_adr, wait_for_adr, wait_for_adr, wait_for_adr, wait_for_adr, wait_for_adr, wait_for_adr},
/* adr_received */		{wait_id,      NULL,         NULL,         NULL,         NULL,         NULL,         NULL        },
/* ID_received */		{NULL,         wait_length,  NULL,         NULL,         NULL,         NULL,         NULL        },
/* length_received */	{NULL,         NULL,         wait_data,    NULL,         NULL,         NULL,         NULL        },
/* parse_data */		{NULL,         NULL,         NULL,         wait_data,    NULL,         NULL,         NULL        },
/* data_received */		{NULL,         NULL,         NULL,         wait_crc,     NULL,         NULL,         NULL        },
/* CRC8_failed */		{NULL,         NULL,         NULL,         NULL,         send_NACK,    NULL,         NULL        },
/* CRC8_ok */           {NULL,         NULL,         NULL,         NULL,         process_msg,  NULL,         NULL        }
};

/**
 * Global variables
 */
/** Data structure that will be used to parse received data into message */
packet_t packet_in = {0, 0, 0, {0}};
packet_t packet_out = {0, 0, 0, {0}};
/** State machine state and transition variables */
state_e s;
transition_e e;
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
static state_e wait_for_adr(transition_e *e)
{
	uint8_t c;

    /* If there is no data available, return */
    if (uart_bkbhit())
    {
		/* Check for the data */
		if (uart_bgetc(&c))
		{
			/** Store device ADDRESS to the packet buffer */
			packet_in.DEV_ADR = c;
			rx_count++;
			/** Go to next state */
			*e = adr_received;
		}
    }

    return PARSE_ADDRESS;
}

/**
 * Callback that read third character identifier, stores it in data_id
 */
static state_e wait_id(transition_e *e)
{
	uint8_t c;

    /* If there is no data available, return */
    if (uart_bkbhit())
    {
		/* Check for the data */
		if (uart_bgetc(&c))
		{
			/** Store device ADDRESS to the packet buffer */
			packet_in.ID = c;
			rx_count++;
			/** Go to next state */
			*e = ID_received;
		}
    }

    return PARSE_ID;
}

/**
 * Callback that read protocol data length
 */
static state_e wait_length(transition_e *e)
{
	uint8_t c;

    /* If there is no data available, return */
    if (uart_bkbhit())
    {
		/* Check for the data */
		if (uart_bgetc(&c))
		{
			/** Store device ADDRESS to the packet buffer */
			packet_in.LENGTH = c;
			rx_count++;
			/** Go to next state */
			*e = length_received;
		}
    }

    return PARSE_LENGTH;
}

/**
 * Callback that read data and store it in packet.DATA buffer
 */
static state_e wait_data(transition_e *e)
{
	uint8_t c;

    /* If there is no data available, return */
    if (uart_bkbhit())
    {
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
				/** Go to next state */
				*e = data_received;	  /* Data has been received, next byte representing checksum */
			}
			else
			{
				/* Stay in current state */
				*e = parse_data;    /* Still there is a data that should be received until checksum is sent */
			}
		}
    }

	return RECEIVE_DATA;
}

/**
 * Callback that read checksum and store it in packet.CRC8
 */
static state_e wait_crc(transition_e *e)
{
	uint8_t c;

    /* If there is no data available, return */
    if (uart_bkbhit())
    {
		/** Check for the data */
		if (uart_bgetc(&c))
		{
			packet_in.DATA[rx_count - PACKET_HEADER_SIZE] = c;
			/** Check CRC */
			if (packet_in.DATA[rx_count - PACKET_HEADER_SIZE] != crc8(&packet_in, rx_count))
			{
				*e = CRC8_failed;
			}
			else
			{
				*e = CRC8_ok;
			}
		}
    }

	return PARSE_CRC8;
}

/**
 * Callback that sends NACK if received checksum is failed
 * @param e[in] transition to next state state
 * @return current state machine state
 */
static state_e send_NACK(transition_e *e)
{
	rx_count = 0;
	NEW_MSG_RECEIVED_FLAG = false;
	/*
	 * NACK and request for retransmission should be coded here
	 * printf(uart_bputc, "CRC failed!\r\n");
	 */

	/* Reset state machine */
	*e = reset_sm;
	return SEND_NACK;
}

/**
 * Callback that initiate message processing
 * @param e[in] transition to next state state
 * @return current state machine state
 */
static state_e process_msg(transition_e *e)
{
	rx_count = 0;
	NEW_MSG_RECEIVED_FLAG = true;

	/* Reset state machine */
	*e = reset_sm;

	return PROCESS_MSG;
}

/**
 * Function apply event to the state machine and call appropriate callback function
 * @param s[in] current state machine state
 * @param e[in] current transition event
 * @return true if state machine error has been detected, false if there is no error
 */
static bool sm_apply_event(state_e *s, transition_e *e)
{
	bool sm_error = false;

	if (*s >= N_STATES || *e >= N_TRANSITIONS)
	{
		sm_error = true;
	}
	else
	{
		*s = state_trans[*e][*s].state_fptr(e);
		sm_error = false;
	}

	return sm_error;
}

/**
 * Public functions
 */

void protocol_init_state_machine()
{
	s = PARSE_ADDRESS;
	e = reset_sm;

	rx_count = 0;
	NEW_MSG_RECEIVED_FLAG = false;
}

bool protocol_parse_uart_data()
{
	bool msg_rcv_done = false;
    /** check state machine */
	if (sm_apply_event(&s, &e))
	{
		/*
		 *  State machine error should be handled here
		 *  printf(uart_bputc, "SM Error!\r\n");
		 */
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
