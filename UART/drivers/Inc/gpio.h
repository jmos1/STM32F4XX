/*
 * 			Author     : James M
 * 			File       : gpio.h
 *
 * 			Description: Header file for interacting w/ GPIOx peripherals
 *
 */
#ifndef _GPIO_H_
#define _GPIO_H_

#include "stm32f446xx.h"

/* ---MODER register bits--- */
#define GPIOx_MODER_Px7_BIT 	14 // 2-bit wide PortX P7 mode. 0b00 = input, 0b01 = output, 0b10 = alt, 0b11 = analog.
#define GPIOx_MODER_Px6_BIT 	12 // 2-bit wide PortX P6 mode. 0b00 = input, 0b01 = output, 0b10 = alt, 0b11 = analog.
#define GPIOx_MODE_ALT 	  	  0b10

/* ---AFRL register bits--- */
#define GPIOx_AFRL6_BIT 	    24 // 4-bit wide PortX P7 pin function. 0b0000 = AF0, 0b0001 = AF1... 0b1111 = AF15
#define GPIOx_AFRL7_BIT 	    28 // 4-bit wide PortX P6 pin function. 0b0000 = AF0, 0b0001 = AF1... 0b1111 = AF15
#define GPIOx_ALTFUNC_AF8 	0b1000

/* ---OTYPER register bits--- */
#define GPIOx_OTYPER_OT6_BIT  6 // P6 output type. 0 = pushpull, 1 = open drain.
#define GPIOx_OTYPER_OT7_BIT  7 // P7 output type. 0 = pushpull, 1 = open drain.
#define GPIOx_OTYPE_PUSHPULL  0
#define GPIOx_OTYPE_ODRAIN    1

/* ---PUPDR register bits--- */
#define GPIOx_PUPDR_P6_BIT   12 // 2-bit wide P6 pull type. 0b00 = no pull, 0b01 = pullup, 0b10 = pulldown, 0b11 = reserved.
#define GPIOx_PUPDR_P7_BIT   14 // 2-bit wide P7 pull type. 0b00 = no pull, 0b01 = pullup, 0b10 = pulldown, 0b11 = reserved.
#define GPIOx_PULLTYPE_UP  0b01

#endif /* _GPIO_H_ */

