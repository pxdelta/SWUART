# SWUART
Port of AN2781 UART emulation software in STM8S and STM8A microcontrollers to IAR

This is a port of
AN2781 UART emulation software in STM8S and STM8A microcontrollers
to IAR without external library (using IAR STM8 definitions of SFR registers)
Compiled with IAR Embedded Workbench for STMicroelectronics STM8 version 2.20.2
Tested on
STM8S105C6T6 microcontroller, 32 KB Flash, 2 KB RAM, 1 KB EEPROM

In tests, I get some reception errors when using TIM4 with fixed time interruption in 1ms

Joel Langer 2018 joellanger@gmail.com
