/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/



#include "stm32f4xx.h"
#include "definitions.h"
#include "console_user.h"
#include "protocol.h"
#include "gpio_user.h"
#include "delay.h"

int main(void)
{
	uint8_t c;
	uint8_t d[2] = {0x00, 0x01};

	delay_init();

#if defined(DEBUG_CONSOLE) || defined(INFO_CONSOLE)
	console_init();
#endif
	uart_init(115200);
	gpio_init();
	GPIO_WriteBit(GPIOC, GPIO_Pin_8, Bit_RESET);

	//uart_write(d, 2);
	//protocol_send_message(d, 2, 2);
	while (1)
	{
		if(protocol_parse_uart_data())
		{
			/** The message is received */
		}

		GPIO_WriteBit(GPIOC, GPIO_Pin_8, Bit_SET);
		delay_ms(500);
		GPIO_WriteBit(GPIOC, GPIO_Pin_8, Bit_RESET);
		delay_ms(500);
	}
}
