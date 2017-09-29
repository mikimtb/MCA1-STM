/*
 * device.h
 *
 *  Created on: Apr 9, 2017
 *      Author: Miroslav
 */

#ifndef DEVICE_H_
#define DEVICE_H_

#include <stm32f4xx.h>

/**
 * Callback function type definition
 */
typedef void (*cmdHdlr_t)(void);

/**
 * Enum definition
 */
/**
 * COMMAND ID enumeration
 * Note:
 * Add as much commands as you need. For each added command callback handler must be defined
 */
typedef enum
{
    CMD_GET_FIRMWARE_VERSION = 1,

} cmdID_e;

/**
 * Type definition
 */
/**
 * Command type definition
 */
typedef struct cmd_info
{
    cmdID_e cmd_id;             /** Command ID */
    cmdHdlr_t   cmd_hdlr;       /** Command callback handler */
} cmdInfo_t;

/**
 * Function prototypes
 */
/**
 * Function parse command and call appropriate callback function
 * @param[in] cmd_id Command identifier
 * @return 0 if callback handler is found, -1 if there is no callback handler for given command ID
 */
int device_cmd_parse_request(uint8_t cmd_id);
/**
 * Callback that response on command CMD_GET_FIRMWARE_VERSION
 */
void device_get_firmware_version_hdlr();

#endif /* DEVICE_H_ */
