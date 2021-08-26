/*
 *					 Author: James M
 *					 File  : uart.c
 *
 * 					 USART6 interrupt-based driver specifications
 *
 * 					 PC6 		  - USART6_Tx
 * 					 PC7 		  - USART6_Rx
 * 					 Baud rate 	  - 9600
 * 					 Data bits 	  - 8
 * 					 Stop bits 	  - 1
 * 					 Parity       - 0
 * 					 Sampling 	  - 16 bits
 * 					 HW flow ctrl - Disabled
 */
#include "uart.h"
#include "gpio.h"

/* ---Global USART6 buffers for application to use--- */
char gRxBuf[USART6_TXBUF_SIZE];
char gTxBuf[USART6_RXBUF_SIZE] = "Message Received.\n\r";
uint8_t gRxBufIter = 0;
uint8_t gTxBufIter = 0;


/*
 * 				Description: Initialization function to set up UART6 w/ interrupts
 */
void usart6_init() {
	/* ---GPIOC register configurations--- */
	// Enable GPIOC peripheral clock
	RCC->AHB1ENR |= (EN << RCC_AHB1ENR_BIT);
	// Set GPIO PC6 and PC7 mode as alternate
	GPIOC->MODER |= (GPIOx_MODE_ALT << GPIOx_MODER_Px7_BIT);
	GPIOC->MODER |= (GPIOx_MODE_ALT << GPIOx_MODER_Px6_BIT);
	// Set alternate function of pins to USART6
	GPIOC->AFRL |= (GPIOx_ALTFUNC_AF8 << GPIOx_AFRL7_BIT);
	GPIOC->AFRL |= (GPIOx_ALTFUNC_AF8 << GPIOx_AFRL6_BIT);
	// Set pullup resistor on PC7 and PC6
	GPIOC->PUPDR |= (GPIOx_PULLTYPE_UP << GPIOx_PUPDR_P6_BIT);
	GPIOC->PUPDR |= (GPIOx_PULLTYPE_UP << GPIOx_PUPDR_P7_BIT);

	/* ---NVIC-based register configurations--- */
	// Enable interrupts for USART6(line 71)
	NVIC_ISER->ISER2 |= (EN << NVIC_ISER_USART6_BIT);
	// Set priority of IRQ71 to 78(Can be anything > 15)
	NVIC_IPR->IPR17 |= (IRQ_PRIORITY78 << NVIC_IPR_POS71_BIT);

	/* ---USART6 register configurations--- */
	// Enable USART6 peripheral clk
	USART6_CLK_EN();
	// Enable TX and RX functionality
	USART6->CR1 |= (EN << USART_CR1_RXEN_BIT);
	USART6->CR1 |= (EN << USART_CR1_TXEN_BIT);
	// Disable parity
	USART6->CR1 &= ~( !(DI) << USART_CR1_PARITY_CTRL_BIT);
	// Set 1 stop bit
	USART6->CR2 |= (USART_STOP_BITS1 << USART_CR2_STOP_BITS);
	// Disable RTS & CTS HW flow control
	USART6->CR3 &= ~(EN << USART_CR3_RTS_CTRL_BIT);
	USART6->CR3 &= ~(EN << USART_CR3_CTS_CTRL_BIT);
	// Set oversampling to 16
	USART6->CR1 &= ~( !(USART_OVERSAMPLE16) << USART_CR1_OVERSAMPLE_BIT );
	// Set baud rate to 9600. Calculation for MANTISSA.DIV = (FCLK/NO_SAMPLES)/BAUDRATE.
	// For this application MANTISSA.DIV = (16MHz/16)/9600 = 104.17
	// MANTISSA = 104 and DIV = 0.17 * NO_SAMPLES = 3
	USART6->BRR |= (USART_9600_MANTISSA << USART_BRR_MANTISSA_BITS);
	USART6->BRR |= (USART_9600_DIV << USART_BRR_DIV_BITS);
	// Enable peripheral USART6
	USART6->CR1 |= (EN << USART_CR1_UE_BIT);
}


/*
 * 				Description: Interrupt-based implementation of Rx and Tx for USART6
 *
 *				usart6_receive() - On Rx reg full interrupt(RXNE), R 1 byte into global Rx buffer. Increment iterator.
 *				usart6_send() 	 - On Tx reg empty interrupt(TXE), W 1 byte into global Tx buffer. Increment iterator.
 *
 */
void usart6_receive() {
	gRxBuf[gRxBufIter] = USART6->DR;
	gRxBufIter++;
}

void usart6_send() {
	USART6->DR = gTxBuf[gTxBufIter];
	gTxBufIter++;
}


/*
 * 				Description: IRQ handler function for USART6 Rx reg full and Tx reg empty events
 *
 */
void usart6_irq_handler() {
	// CR1 RXNEIE bit enabled
	if( ((USART6->CR1) >> USART_CR1_RXNEIE_BIT) & EN ) {
		usart6_receive();
	}
	// CR1 TXEIE bit enabled
	else if( ((USART6->CR1) >> USART_CR1_TXIE_BIT) & EN ) {
		usart6_send();
	}
}


/*
 *			    Description: Enable and disable interrupt triggers on UART6
 *
 * 				usart6_tx_irqen() - ENABLE Tx data register EMPTY flag interrupt
 * 				usart6_rx_irqen() - ENABLE Rx data register FULL flag interrupt
 * 				usart6_tx_irqdi() - DISABLE " 	"
 * 				usart6_rx_irqdi() - DISABLE "	"
 *
 */
void usart6_tx_irqen() {
	USART6->CR1 |= ( EN << USART_CR1_TXIE_BIT );
}

void usart6_rx_irqen() {
	USART6->CR1 |= ( EN << USART_CR1_RXNEIE_BIT );
}

void usart6_tx_irqdi() {
	USART6->CR1 &= ~( !(DI) << USART_CR1_TXIE_BIT );
}

void usart6_rx_irqdi() {
	USART6->CR1 &= ~( !(DI) << USART_CR1_RXNEIE_BIT );
}

