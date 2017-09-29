/*
 * device.c
 *
 *  Created on: Apr 9, 2017
 *      Author: Miroslav
 */
#include "device.h"

/**
 * User variables
 * Note:
 * For each command ID callback handler should be defined
 */
static cmdInfo_t cmd_hdlrs[] =
{
    {CMD_GET_FIRMWARE_VERSION, device_get_firmware_version_hdlr}
};

/**
 * Function definition
 */

int device_cmd_parse_request(uint8_t cmd_id)
{
    int i=0;
    /**
     * Get cmd_hdlrs_array length
     */
    int cmd_hdlrs_count = sizeof(cmd_hdlrs) / sizeof(cmd_hdlrs[0]);
    /**
     * Find handler for given message ID
     */
    while(i < cmd_hdlrs_count && cmd_hdlrs[i].cmd_id != cmd_id)
    {
        i++;
    }
    /**
     * If there is no proper handler return error code -1
     */
    if (i == cmd_hdlrs_count)
        return -1;
    /**
     * Callback calling
     */
    (*cmd_hdlrs[i].cmd_hdlr)();

    return 0;
}
