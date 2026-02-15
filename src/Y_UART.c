#include "inc/Y_UART.h"

/* max uart output message length */
static const uint8_t UART2_buffer_len = 128;

/* uart tx buffer */
static uint8_t UART2_BUFFER[128];
static uint8_t UART2_MSG_LENGHT = 0;
static uint8_t UART2_START_INDEX = 0;
static uint8_t UART2_buffer_pointer = 127;

/* add 32-bit integer to buffer */
__attribute__((section(".ram_code")))
void UART2_add_to_buffer(int32_t i)
{
    uint8_t is_negative = 0;
    if(i < 0)
    {
        i *= -1;
        is_negative = 1;
    }
    do
    {
    	UART2_BUFFER[UART2_buffer_pointer] = '0' + (i % 10);
    	UART2_buffer_pointer--;
        i /= 10;
    }while (i != 0);

    if(is_negative)
    {
    	UART2_BUFFER[UART2_buffer_pointer] = '-';
    	UART2_buffer_pointer--;
    }

    UART2_BUFFER[UART2_buffer_pointer] = ' ';
    UART2_buffer_pointer--;
}

__attribute__((section(".ram_code")))
void UART2_reset_buffer()
{
	/* reset buffer pointer to start */
	UART2_buffer_pointer = UART2_buffer_len - 1;

	/* add end line */
	UART2_BUFFER[UART2_buffer_pointer] = '\n';
	UART2_buffer_pointer--;
}

__attribute__((section(".ram_code")))
void UART2_transmit_buffer()
{
	UART_STATUS_t uart_status;

	UART2_MSG_LENGHT = UART2_buffer_len - UART2_buffer_pointer - 1;
	UART2_START_INDEX = UART2_buffer_len - UART2_MSG_LENGHT;

	while(UART_0.runtime->tx_busy);
	uart_status = UART_Transmit(&UART_0, (uint8_t*)(UART2_BUFFER+UART2_START_INDEX), UART2_MSG_LENGHT);
	while(UART_0.runtime->tx_busy);
}
