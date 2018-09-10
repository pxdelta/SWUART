#include "derivative.h"
#include <stdint.h>
#include "swuart.h"

/*
This is a port of
AN2781 UART emulation software in STM8S and STM8A microcontrollers
to IAR without external library (using IAR STM8 definitions of SFR registers)
Compiled with IAR Embedded Workbench for STMicroelectronics STM8 version 2.20.2
Tested on
STM8S105C6T6 microcontroller, 32 KB Flash, 2 KB RAM, 1 KB EEPROM

Joel Langer 2018 joellanger@gmail.com
*/

#pragma vector = TIM3_OVR_UIF_vector
__interrupt void TIM3_UPD_OVF_BRK_IRQHandler(void)
{
    #ifdef SWUART_TRANSMIT_USED
    uart_Tx_timing();
    #endif
}

#pragma vector = TIM3_CAPCOM_CC1IF_vector
__interrupt void TIM3_CAP_COM_IRQHandler(void)
{
    #ifdef SWUART_RECEIVE_USED
    uart_Rx_timing();
    #endif
}

uint8_t tx_byte=0xAA;
uint8_t buff, sts;
uint8_t rxerr=0;

void Delay(void)
{
    uint8_t t;
    for(t=0;t<0xFF;t++)
    {
        asm("NOP");
    }
}

//CPU at 16MHz
void main(void)
{
    __disable_interrupt();
    
    //Fmaster = 16MHz
    //HSI
    CLK_CKDIVR=(MASK_CLK_CKDIVR_HSIDIV&0);//prescaler = 1
    
	uart_init();							// init pins and variables of SW UART
	uart_receive_enable;					// enable receive process
    
    __enable_interrupt();
    
    for(;;)
    {
        ++tx_byte;												// data byte change
        sts= uart_send(tx_byte);							// send a byte
        while(test_status(transmit_in_progress) != 0);// wait for its transmition complete
        Delay();
        sts= uart_read(&buff);								// read receive buffer
        if(sts != receive_buffer_full	|| buff != tx_byte) // check content of the status & data registers
        rxerr++;								// LEDs off if the data byte isn't correctly received
        Delay();
        Delay();
        Delay();
    }
}
