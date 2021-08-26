/*
 * 			Author	   : James M
 * 			File  	   : stm32f446xx.h
 *
 * 			Description: Header file for memory boundaries and register structures on STM32f446XX MCUs
 *
 */
#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include <stdint.h>
#include "uart.h"

// |******************************************************************************************| //
// |***************************************MEMORY BOUNDARIES**********************************| //
// |******************************************************************************************| //
/* ---Cortex-M4 processor registers--- */
#define MEMORY_BEGIN_NVIC_ISER (uint32_t)0xE000E100
#define MEMORY_BEGIN_NVIC_ICER (uint32_t)0xE000E180
#define MEMORY_BEGIN_NVIC_IPR  (uint32_t)0xE000E400

/* ---AHB1 bus registers--- */
#define MEMORY_BEGIN_GPIOA (uint32_t)0x40020000
#define MEMORY_BEGIN_GPIOB (uint32_t)0x40020400
#define MEMORY_BEGIN_GPIOC (uint32_t)0x40020800
#define MEMORY_BEGIN_GPIOD (uint32_t)0x40020C00
#define MEMORY_BEGIN_GPIOE (uint32_t)0x40021000
#define MEMORY_BEGIN_GPIOF (uint32_t)0x40021400
#define MEMORY_BEGIN_GPIOG (uint32_t)0x40021800
#define MEMORY_BEGIN_GPIOH (uint32_t)0x40021C00

/* ---APB2 bus registers--- */
#define MEMORY_BEGIN_RCC    (uint32_t)0x40023800
#define MEMORY_BEGIN_EXTI   (uint32_t)0x40013C00
#define MEMORY_BEGIN_SYSCFG (uint32_t)0x40013800
#define MEMORY_BEGIN_USART6 (uint32_t)0x40011400
#define MEMORY_BEGIN_USART1 (uint32_t)0x40011000

/* ---APB1 bus registers--- */
#define MEMORY_BEGIN_UART5 	(uint32_t)0x40005000
#define MEMORY_BEGIN_UART4 	(uint32_t)0x40004C00
#define MEMORY_BEGIN_USART3 (uint32_t)0x40004800
#define MEMORY_BEGIN_USART2 (uint32_t)0x40004400

// |*****************************************************************************************| //
// |***************************************REGISTER STRUCTS**********************************| //
// |*****************************************************************************************| //
/*
 * 						Description: Reset and clock control registers. Used to EN/DI peripheral clocks
 * 									 as well as to configure clock frequency, prescalar, and PLL.
 *
 * 										RCC REGISTERS
 *
 * 										|CLOCK CONTROL            			| -> 0x00
 * 									    |PLL CONFIG      					| -> 0x04
 * 										|CLOCK CONFIG     					| -> 0x08
 * 										|CLOCK INTERRUPT  					| -> 0x0C
 *										|AHB1 PERIPHERAL RESET       		| -> 0x10
 * 										|AHB2 PERIPHERAL RESET      		| -> 0x14
 * 	 									|AHB3 PERIPHERAL RESET    			| -> 0x18
 *  									|APB1 PERIPHERAL RESET 				| -> 0x20
 *  									|APB2 PERIPHERAL RESET 				| -> 0x24
 * 										|AHB1 PERIPHERAL CLK EN				| -> 0x30
 * 										|AHB2 PERIPHERAL CLK EN				| -> 0x34
 * 										|AHB3 PERIPHERAL CLK EN				| -> 0x38
 * 										|APB1 PERIPHERAL CLK EN				| -> 0x40
 * 										|APB2 PERIPHERAL CLK EN				| -> 0x44
 * 										|AHB1 PERIPHERAL CLK EN (LOW POWER)	| -> 0x50
 * 										|AHB2 PERIPHERAL CLK EN (LOW POWER)	| -> 0x54
 * 										|AHB3 PERIPHERAL CLK EN (LOW POWER)	| -> 0x58
 * 										|APB1 PERIPHERAL CLK EN (LOW POWER)	| -> 0x60
 * 										|APB2 PERIPHERAL CLK EN (LOW POWER)	| -> 0x64
 * 										|BACKUP DOMAIN CONTROL				| -> 0x70
 * 										|CLK CONTROL & STATUS				| -> 0x74
 * 										|SPREAD CLK GENERATION				| -> 0x80
 * 										|PLLI2S CONFIG						| -> 0x84
 * 										|PLL CONFIG							| -> 0x88
 * 										|RCC DEDICATED CLK CONFIG			| -> 0x8C
 * 										|RCC CLK GATED CONFIG				| -> 0x90
 * 										|RCC DEDICATED CLK CONFIG2			| -> 0x94
 *
 *
 */
typedef struct {
	uint32_t volatile CR;
	uint32_t volatile PLLCFGR;
	uint32_t volatile CFGR;
	uint32_t volatile CIR;
	uint32_t volatile AHB1RSTR;
	uint32_t volatile AHB2RSTR;
	uint32_t volatile AHB3RSTR;
	uint32_t RESERVED1;
	uint32_t volatile APB1RSTR;
	uint32_t volatile APB2RSTR;
	uint32_t RESERVED2;
	uint32_t RESREVED3;
	uint32_t volatile AHB1ENR;
	uint32_t volatile AHB2ENR;
	uint32_t volatile AHB3ENR;
	uint32_t RESREVED4;
	uint32_t volatile APB1ENR;
	uint32_t volatile APB2ENR;
	uint32_t RESERVED5;
	uint32_t RESREVED6;
	uint32_t volatile AHB1LPENR;
	uint32_t volatile AHB2LPENR;
	uint32_t volatile AHB3LPENR;
	uint32_t RESREVED7;
	uint32_t volatile APB1LPENR;
	uint32_t volatile APB2LPENR;
	uint32_t RESERVED8;
	uint32_t RESREVED9;
	uint32_t volatile BDCR;
	uint32_t volatile CSR;
	uint32_t RESERVED10;
	uint32_t RESREVED11;
	uint32_t volatile SSCGR;
	uint32_t volatile PLLI2SCFGR;
	uint32_t volatile PLLSAICFGR;
	uint32_t volatile DCKCFGR;
	uint32_t volatile CKGATENR;
	uint32_t volatile DCKCFGR2;
} RCC_REGS_t;

#define RCC ( (RCC_REGS_t*)(MEMORY_BEGIN_RCC) )
#define RCC_AHB1ENR_BIT 2


/*
 * 				Description: GPIOx peripheral registers. Use to configure IO as well as alternate functions
 * 							 such as PWM, ADC, USART, SPI, I2C, CAN...etc protocols.
 *
 * 										GPIOx PERIPHERAL REGISTERS
 *
 * 										|MODE             | -> 0x00
 * 									    |OUTPUT TYPE      | -> 0x04
 * 										|OUTPUT SPEED     | -> 0x08
 * 										|PULLUP/PULLDOWN  | -> 0x0C
 *										|INPUT DATA       | -> 0x10
 * 										|OUTPUT DATA      | -> 0x14
 * 	 									|PORT BIT S\R     | -> 0x18
 *  									|PORT CONFIG LOCK | -> 0x1C
 *  									|ALT FUNCTION LOW | -> 0x20
 * 										|ALT FUNCTION HIGH| -> 0x24
 *
 */
typedef struct {
	uint32_t volatile MODER;
	uint32_t volatile OTYPER;
	uint32_t volatile OSPEEDR;
	uint32_t volatile PUPDR;
	uint32_t volatile IDR;
	uint32_t volatile ODR;
	uint32_t volatile BSRR;
	uint32_t volatile LCKR;
	uint32_t volatile AFRL;
	uint32_t volatile AFRH;
} GPIO_REGS_t;

#define GPIOC ( (GPIO_REGS_t*)(MEMORY_BEGIN_GPIOC) )


/*
 * 				Description: USART peripheral registers. Use to configure and R/W to and from the peripheral
 *
 * 										USART REGISTERS
 *
 * 										|STATUS 			     | -> 0x00
 * 									    |DATA 				     | -> 0x04
 * 										|BAUD RATE 				 | -> 0x08
 * 										|CONTROL REGISTER 1		 | -> 0x0C
 *										|CONTROL REGISTER 2		 | -> 0x10
 * 										|CONTROL REGISTER 3		 | -> 0x14
 * 										|GUARD TIME AND PRESCALAR| -> 0x18
 *
 */
typedef struct {
	uint32_t volatile SR;
	uint32_t volatile DR;
	uint32_t volatile BRR;
	uint32_t volatile CR1;
	uint32_t volatile CR2;
	uint32_t volatile CR3;
	uint32_t volatile GTPR;
} USART_REGS_t;

#define USART6 ( (USART_REGS_t*)(MEMORY_BEGIN_USART6) )


/*
 * 				Description. EXTI interrupt line configs. 23 in total on an STM32F44XX system. Use for
 * 							 EN/DI GPIO interrupts as well as whatever other peripherals specified in the TRM.
 * 							 EXTI lines 0 - 15 are for pins 0 - 15 on GPIO port X.
 * 							 Lines 5 - 9 share 1 NVIC position. Lines 10-15 share 1 NVIC position.
 * 							 Lines 0 - 5 have their own discrete NVIC positions.
 * 							 ***PR REGISTER MUST be cleared after an external hardware interrupt***
 *
 * 										EXTI INTERRUPT REGISTERS
 *
 * 										|INTERRUPT MASK          | -> 0x00
 * 									    |EVENT MASK              | -> 0x04
 * 										|RISING TRIGGER SELECT   | -> 0x08
 * 										|FALLING TRIGGER SELECT  | -> 0x0C
 *										|SOFTWARE INTERRUPT EVENT| -> 0x10
 * 										|PENDING  		         | -> 0x14
 *
 */
typedef struct {
	uint32_t volatile IMR;
	uint32_t volatile EMR;
	uint32_t volatile RSTSR;
	uint32_t volatile FTSR;
	uint32_t volatile SWIER;
	uint32_t volatile PR;
} EXTI_REGS_t;

#define EXTI ( (EXTI_REGS_t*)(MEMORY_BEGIN_EXTI) )


/*
 * 				Description: Used for memory remapping in code area as well as managing EXTI line connections
 * 							 to GPIO pins. Since 16 pins are muxed through one single EXTI line, you must use
 * 							 EXTI CONFIG registers to choose which port is selected for the pin of interest.
 *
 * 										SYSCFG REGISTERS
 *
 * 										|MEMORY REMAP          | -> 0x00
 * 									    |PERIPHERAL MODE CONFIG| -> 0x04
 * 										|EXTI CONFIG 1  	   | -> 0x08
 * 										|EXTI CONFIG 2		   | -> 0x0C
 *										|EXTI CONFIG 3		   | -> 0x10
 * 										|EXTI CONFIG 4	       | -> 0x14
 * 										|CMPCR        	       | -> 0x20
 * 										|CFGR	        	   | -> 0x2C
 *
 */
typedef struct {
	uint32_t volatile MEMRMP;
	uint32_t volatile PMC;
	uint32_t volatile EXTICR1;
	uint32_t volatile EXTICR2;
	uint32_t volatile EXTICR3;
	uint32_t volatile EXTICR4;
	uint32_t RESERVED1;
	uint32_t RESERVED2;
	uint32_t volatile CMPCR;
	uint32_t RESERVED3;
	uint32_t RESERVED4;
	uint32_t volatile CFGR;
} SYSCFG_REGS_t;


#define SYSCFG ( (SYSCFG_REGS_t*)(MEMORY_BEGIN_SYSCFG) )


/*
 * 				Description: Interrupt set enable registers to enable processor interrupts.
 * 							 Each bit corresponds to an interrupt position in the vector table.
 *
 * 										NVIC ISER REGISTERS
 *
 * 										|ISER0| -> 0x00
 * 									    |ISER1| -> 0x04
 * 										|ISER2| -> 0x08
 * 										|ISER3| -> 0x0C
 *										|ISER4| -> 0x10
 * 										|ISER5| -> 0x14
 * 										|ISER6| -> 0x18
 * 										|ISER7| -> 0x1C
 *
 */
typedef struct {
	uint32_t volatile ISER0;
	uint32_t volatile ISER1;
	uint32_t volatile ISER2;
	uint32_t volatile ISER3;
	uint32_t volatile ISER4;
	uint32_t volatile ISER5;
	uint32_t volatile ISER6;
	uint32_t volatile ISER7;
} NVIC_ISER_REGS_t;

#define NVIC_ISER ( (NVIC_ISER_REGS_t*)(MEMORY_BEGIN_NVIC_ISER) )
#define NVIC_ISER_USART6_BIT 7 // USART6 bit position on ISER2 to EN position 71 interrupts


/*
 *
 * 				Description: Interrupt clear enable registers. Use to disable an interrupt that
 * 							 was set with the ISER register.
 *
 * 										NVIC ICER REGISTERS
 *
 * 										|ICER0| -> 0x00
 * 									    |ICER1| -> 0x04
 * 										|ICER2| -> 0x08
 * 										|ICER3| -> 0x0C
 *										|ICER4| -> 0x10
 * 										|ICER5| -> 0x14
 * 										|ICER6| -> 0x18
 * 										|ICER7| -> 0x1C
 *
 */
typedef struct {
	uint32_t volatile ICER0;
	uint32_t volatile ICER1;
	uint32_t volatile ICER2;
	uint32_t volatile ICER3;
	uint32_t volatile ICER4;
	uint32_t volatile ICER5;
	uint32_t volatile ICER6;
	uint32_t volatile ICER7;
} NVIC_ICER_REGS_t;

#define NVIC_ICER ( (NVIC_ICER_REGS_t*)(MEMORY_BEGIN_NVIC_ICER) )


/*
 * 				Description: Interrupt priority registers. Each reg is divided into 8-bit chunks used to
 * 							 set the priority of interrupts corresponding to a position. 59 registers in total -
 * 							 only 20 defined in the struct below.
 *
 * 										NVIC IPR REGISTERS
 *
 * 										|IPR0 | -> 0x00
 * 									    |IPR1 | -> 0x04
 * 										|IPR2 | -> 0x08
 * 										|IPR3 | -> 0x0C
 *										|IPR4 | -> 0x10
 * 										|IPR5 | -> 0x14
 * 										|IPR6 | -> 0x18
 * 										|IPR7 | -> 0x1C
 * 										   .
 * 										   .
 *										   .
 *										|IPR59|
 *
 */
typedef struct {
	uint32_t volatile IPR0;
	uint32_t volatile IPR1;
	uint32_t volatile IPR2;
	uint32_t volatile IPR3;
	uint32_t volatile IPR4;
	uint32_t volatile IPR5;
	uint32_t volatile IPR6;
	uint32_t volatile IPR7;
	uint32_t volatile IPR8;
	uint32_t volatile IPR9;
	uint32_t volatile IPR10;
	uint32_t volatile IPR11;
	uint32_t volatile IPR12;
	uint32_t volatile IPR13;
	uint32_t volatile IPR14;
	uint32_t volatile IPR15;
	uint32_t volatile IPR16;
	uint32_t volatile IPR17;
	uint32_t volatile IPR18;
	uint32_t volatile IPR19;
	uint32_t volatile IPR20;
	// Add more registers if needed
} NVIC_IPR_REGS_t;

#define NVIC_IPR ( (NVIC_IPR_REGS_t*)(MEMORY_BEGIN_NVIC_IPR) )
#define NVIC_IPR_POS71_BIT 24 // Interrupt priority bits for USART6
#define IRQ_PRIORITY78     78


// |*****************************************************************************************| //
// |********************************************MACROS***************************************| //
// |*****************************************************************************************| //
#define EN    1
#define DI 	  0
#define SET   1
#define RESET 0

#endif /* INC_STM32F446XX_H_ */
