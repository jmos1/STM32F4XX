/*
 * 			Author     : James M
 * 			File       : uart.h
 *
 * 			Description: Header file for utilizing UART6 peripheral and uart.c driver file
 *
 */
#ifndef INC_UART_H_
#define INC_UART_H_

#include "stm32f446xx.h"
#include <stdint.h>

/* ---CR1 register bits--- */
#define USART_CR1_PARITY_CTRL_BIT 10 // Parity ctrl bit. 1 to enable. 0 to disable
#define USART_CR1_TXEN_BIT 		   3 // Tx enable bit. 1 to enable. 0 to disable
#define USART_CR1_RXEN_BIT         2 // Rx enable bit. 1 to enable. 0 to disable
#define USART_CR1_M_BIT            2 // Word length bit. 0 for 8 data/1 start bit. 1 for 9 data/1 start bit

#define USART_CR1_OVERSAMPLE_BIT  16 // Oversample bit. 0 = 16 samples, 1 = 8 samples.
#define USART_OVERSAMPLE16 		   0
#define USART_OVERSAMPLE8 		   1

#define USART_CR1_UE_BIT 		  13 // USART enable bit. 1 = enable. 0 = disabled

#define USART_CR1_TXIE_BIT   	   7 // Tx shift reg empty interrupt. 1 = enable. 0 = disable
#define USART_CR1_TCIE_BIT   	   6 // Tx complete interrupt. 1 = enable. 0 = disable
#define USART_CR1_RXNEIE_BIT 	   5 // Rx shift reg full interrupt. 1 = enable, 0 = disable


/* ---CR2 register bits--- */
#define USART_CR2_STOP_BITS 	  13 // Stop bits. 0b00 - 1 bit, 0b01 - 0.5 bits, 0b10 - 2 bits, 0b11 - 1.5 bits
#define USART_STOP_BITS1  	    0b00


/* ---CR3 register bits--- */
#define USART_CR3_RTS_CTRL_BIT 	   8
#define USART_CR3_CTS_CTRL_BIT     9

/* ---SR register bits--- */
#define USART_SR_TXE_BIT 	 	   7 // Read-only Tx data reg empty. 1 = empty. 0 = full.
#define TX_BUF_EMPTY 1

#define USART_SR_TX_DONE_BIT 	   6 // Read-only Tx complete data reg. 1 = transmit complete. 0 = transmit !complete
#define TX_DONE 				   1

#define USART_SR_RXNE_BIT 	       5 // Rx shift reg full flag. 1 = data can be read. 0 = no data in register.
#define RX_BUF_FULL			       1


/* ---BRR register bits--- */
#define USART_BRR_MANTISSA_BITS    4 // Bits 4-15
#define USART_BRR_DIV_BITS 		   0 // Bits 0-3
#define USART_9600_MANTISSA      104
#define USART_9600_DIV 		       3


/* ---Function prototypes--- */
void usart6_init(void);

void usart6_send(void);
void usart6_receive(void);

void usart6_irq_handler(void);

void usart6_tx_irqen(void);
void usart6_rx_irqen(void);

void usart6_tx_irqdi(void);
void usart6_rx_irqdi(void);


/* ---USART specific macros--- */
#define USART6_CLK_EN() ( ((RCC_REGS_t*)(MEMORY_BEGIN_RCC))->APB2ENR |= 0x20 )
#define USART6_TXBUF_SIZE 128
#define USART6_RXBUF_SIZE 128

#endif /* INC_UART_H_ */
