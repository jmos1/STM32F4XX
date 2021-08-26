/*
 *			Author: James M
 *			File: uart_demo.c
 *
 *			Description: USART6 interrupt-based demonstration. User transmits to MCU
 *				    	 via serial console, and MCU acknowledges the reception w/ a message.
 *
 */
#include "stm32f446xx.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* Defined in uart.c */
extern char gRxBuf[USART6_RXBUF_SIZE];
extern char gTxBuf[USART6_TXBUF_SIZE];
extern uint8_t gRxBufIter;
extern uint8_t gTxBufIter;

int main(void)
{
	usart6_init();
	usart6_rx_irqen();

	/* Receive a message from serial terminal and reply back with an acknowledgement message. */
	while(1) {
		if( (gRxBufIter > 0) && (gRxBuf[gRxBufIter-1] == '\r') ) {
			gRxBufIter = 0; // Reset Rx buf index to 0 to avoid overflow during next reception
			usart6_rx_irqdi();
			usart6_tx_irqen();
		}
		else if( (gTxBufIter > 0) && (gTxBuf[gTxBufIter-1] == '\r') ) {
			gTxBufIter = 0; // Reset Tx buf index to 0 to send the same message again.
			usart6_tx_irqdi();
			usart6_rx_irqen();
		}
	}
}

/* ---IRQ handler for USART6--- */
void USART6_IRQHandler() {
	usart6_irq_handler();
}















