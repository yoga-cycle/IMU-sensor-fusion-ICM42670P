/*
 * Helper function for uart transmission
 */

#pragma once
#include <stdint.h>
#include "UART/uart.h"

/* add 32-bit integer to buffer */
void UART2_add_to_buffer(int32_t i);

/* resets buffer to empty state */
void UART2_reset_buffer();

/* transmit buffer */
void UART2_transmit_buffer();
