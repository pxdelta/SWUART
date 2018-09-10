/**
  ******************************************************************************
  * @file swuart.h
  * @brief This file contains all user and system definition of uart software.
  * @author STMicroelectronics - MCD Application Team
  * @version V1.0.0
  * @date 10/13/2008
  ******************************************************************************
  *
  * THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2008 STMicroelectronics</center></h2>
  * @image html logo.bmp
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SW_UART_H
#define __SW_UART_H

#include <stdint.h>
// ---------------------------------------------------------
// --------------- user macros definition ------------------
// ---------------------------------------------------------
#ifndef TRUE 
#define TRUE  (1)
#endif
#ifndef FALSE
#define FALSE (0)
#endif

#define SWUART_TRANSMIT_USED
#define SWUART_RECEIVE_USED
//#define TEST_PIN_USED

#define	DATA_LENGTH		8		// number of data bits in protocol
#define	STOP_BITS		1		// number of stop bits in protocol
//#define  PARITY
// if PARITY flag si defined the most significant data
// 			bit is considered to be parity bit

// user HW definition -------------------------------------
// Macros in following section should be carefully completed by user 
// in according with hardware connection of Tx and Rx pins, used
// protocol and speed (Rx pin is supposed with capture capability,
// Tx pin is any with output capability)
//----------------------------------------------------------

//	Here user must ensure calling service interrupt routine at exact intervals 
//	of 1/2 bit of the dedicated baud rate (see also TO_HSE switch above)
// TIM3 overflow period must be set in dependance on fmaster clock selected:
//                                     fmaster= fcpu clock:   16 / 24 Mhz
// 	for 9600Bd speed set overflow every 52.08us - Period=  833 / 1250
//   for 19200Bd speed set overflow every 26,04us - Period=  417 / 625
//   for 57600Bd speed set overflow every  8,68us - Period=  139*/ 208
// *) for half duplex only!
// next macro initializes and enable TIM3 base and interrupt system, divider
// register /1, auto reload register, enable interrupt on update, update on overflow
#define init_UART_timing	{\
    TIM3_PSCR=MASK_TIM3_PSCR_PSC&0x00;\
    TIM3_ARRH=833>>8;\
    TIM3_ARRL=833&0xFF;\
    TIM3_IER|=MASK_TIM3_IER_UIE;\
	TIM3_CR1|=MASK_TIM3_CR1_URS;\
	TIM3_CR1|=MASK_TIM3_CR1_CEN;\
}

// input capture system initialization
//filter (4 samples), IC2 mapped on TI2FP2, capture on faling edge
#define	init_ic_setting	{\
	TIM3_CCER1&= ~MASK_TIM3_CCER1_CC2E;\
	TIM3_CCMR2= (((2<<4) & MASK_TIM3_CCMR2_OC2M) | ((1<<0) & MASK_TIM3_CCMR2_CC2S));\
	TIM3_CCER1|= MASK_TIM3_CCER1_CC2P | MASK_TIM3_CCER1_CC2E;\
}
// output compare system initialization
// frozen (timing only) mode, preload regs disable, no output
#define  init_oc_setting {\
	TIM3_CCER1&= ~MASK_TIM3_CCER1_CC2E;\
	TIM3_CCMR2= 0;\
}

#define	clear_owerflow_flag	{ TIM3_SR1 &= ~MASK_TIM3_SR1_UIF; }
#define	clear_cc_flag			{ TIM3_SR1 &= ~MASK_TIM3_SR1_CC2IF;	}
#define  enable_cc_interrupt	{ TIM3_IER |=  MASK_TIM3_IER_CC2IE; }
#define  disable_cc_interrupt	{ TIM3_IER &= ~MASK_TIM3_IER_CC2IE; }

#define	enable_IC_system		{ /* clear_cc_flag;*/ init_ic_setting; enable_cc_interrupt; }
#define	disable_IC_system	 {\
	disable_cc_interrupt;\
	TIM3_CCER1 &= (uint8_t)(~MASK_TIM3_CCER1_CC2E);\
	TIM3_CCMR2&=~ MASK_TIM3_CCMR2_CC2S;\
};

#define	enable_OC_system		{ /* clear_cc_flag;*/ init_oc_setting; enable_cc_interrupt; }
#define	disable_OC_system	  	  disable_cc_interrupt

// ---------------------------------------------------------
// ------------- private macros definition -----------------
// ---------------------------------------------------------

#if (DATA_LENGTH == 9)
#ifndef PARITY
#define BIT9					// definition of 9-th data bit flag
#define	set_Tx_bit9		{ Tx_bit9= 1; }
#define	clr_Tx_bit9		{ Tx_bit9= 0; }
#endif
#endif

#if (DATA_LENGTH > 9)				// checking the parameters range
#error DATA LENGTH SHOULD NOT BE LONGER THAN NINE BITS
#endif
#if (DATA_LENGTH < 1)
#error DATA LENGTH SHOULD NOT BE SHORTER THAN ONE BIT
#endif
#if (STOP_BITS > 2)
#error NO MORE THAN TWO STOP BITS SHOULD BE DEFINED
#endif
#if (STOP_BITS < 1)
#error AT LEAST ONE STOP BIT SHOULD BE DEFINED
#endif

// bit manipulation definition
#ifdef SWUART_RECEIVE_USED
#define Rx_test         PD_IDR_IDR0
#endif
#ifdef SWUART_TRANSMIT_USED
#define set_Tx          PD_ODR_ODR2=1
#define clr_Tx          PD_ODR_ODR2=0
#endif
#ifdef TEST_PIN_USED
#define set_Test_Pin
#define clr_Test_Pin
#endif

// initial definition of HW pins Tx as output push-pull, Rx as input floating
 
#ifdef SWUART_TRANSMIT_USED
#define init_UART_Tx_pin    {PD_DDR_DDR2=1;PD_CR1_C12=1;PD_ODR_ODR2=1;}
#endif
#ifdef SWUART_RECEIVE_USED
#define init_UART_Rx_pin    {PD_DDR_DDR0=0;PD_CR1_C10=1;}
#endif
#ifdef TEST_PIN_USED
#define init_UART_Test_pin
#endif

/* Exported constants --------------------------------------------------------*/
// status masks
#define	transmit_in_progress			0x80
#define	transmit_data_reg_empty		0x40
#define	receive_9th_data_bit			0x20
#define	receive_in_progress			0x10

#define	receive_buffer_overflow		0x08 // low nibble corresponds to error return value
#define	receive_frame_error			0x04
#define	receive_noise_error			0x02
#define	receive_buffer_full			0x01

// error return codes
#define OVFL_ERROR	8
#define FRAME_ERROR	4
#define NOISE_ERROR	2
#define RX_BUFF_FULL	1

        
/* Exported macros ------------------------------------------------------------*/
#define test_status(a)		(UART_sts & a)
#define set_status(a)		(UART_sts |= a)
#define clr_status(a)		(UART_sts &=~a)

#define uart_receive_enable 	{ Rx_bit= Rx_phase= 0; disable_OC_system; clear_cc_flag;enable_IC_system; }
#define uart_receive_disable 	{ disable_IC_system; disable_OC_system; clr_status(receive_in_progress); }

/* Exported variables --------------------------------------------------------*/
extern uint8_t Rx_phase;
extern uint8_t Tx_phase;
#ifdef PARITY
extern uint8_t Rx_parity;
extern uint8_t Tx_parity;
#else
#ifdef BIT9
extern uint8_t Rx_bit9;
extern uint8_t Tx_bit9;
#endif
#endif
extern uint8_t Rx_bit,
			 Rx_samp,
			 Tx_bit,
			 Rx_buff,
			 Rx_data,
			 Tx_data,
			 UART_sts;
			 
/* Exported functions ------------------------------------------------------- */
void uart_init(void);
uint8_t uart_send(uint8_t b);
uint8_t uart_read(uint8_t *b);
void uart_Tx_timing(void);
void uart_Rx_timing(void);
#endif

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
