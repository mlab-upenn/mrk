/******************************************************************************
*  Nano-RK, a real-time operating system for sensor networks.
*  Copyright (C) 2007, Real-Time and Multimedia Lab, Carnegie Mellon University
*  All rights reserved.
*
*  This is the Open Source Version of Nano-RK included as part of a Dual
*  Licensing Model. If you are unsure which license to use please refer to:
*  http://www.nanork.org/nano-RK/wiki/Licensing
*
*  This program is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation, version 2.0 of the License.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*  Contributing Authors (specific to this file):
*  Anthony Rowe
*  Nuno Pereira
*  Peter Diener
*******************************************************************************/


#ifndef ULIB_H
#define ULIB_H

#include <nrk_pin_define.h>
#include <nrk_events.h>
#include <stdio.h>

char getc2();
char getc3();
//int putc0(char, FILE * fp);
//int putc1(char, FILE *fp);
void putc2(char x);
void putc3(char x);

uint8_t nrk_uart_data_ready(uint8_t uart_num);

// FIXME: should be using included file
int8_t nrk_uart_rx_signal_get();
//nrk_sig_t nrk_uart_rx_signal_get();
void nrk_kprintf(const char *addr);
void nrk_setup_ports();
void nrk_init_hardware();
void nrk_setup_uart(uint32_t baudrate);
void setup_uart3(uint32_t baudrate);
void setup_uart2(uint32_t baudrate);

//---------------------------------------------------------------------------------------------
// LPC17xx related definitions ///Abhijeet

void initLED(void) ;
void setupSPIPins(void);
void setup_uartPins(void);
void PINSEL_ConfigPin(PINSEL_CFG_Type *PinCfg);
void set_PinFunc ( uint8_t portnum, uint8_t pinnum, uint8_t funcnum);
void set_ResistorMode ( uint8_t portnum, uint8_t pinnum, uint8_t modenum);
void set_OpenDrainMode( uint8_t portnum, uint8_t pinnum, uint8_t modenum);
void GPIO_SetDir(uint8_t portNum, uint32_t bitValue, uint8_t dir);
void GPIO_ClearValue(uint8_t portNum, uint32_t bitValue);
void GPIO_SetValue(uint8_t portNum, uint32_t bitValue);
uint32_t GPIO_ReadValue(uint8_t portNum);
void GPIO_Toggle(uint8_t portNum, uint32_t bitValue);
uint32_t CLKPWR_GetPCLK (uint32_t ClkType);
uint32_t CLKPWR_GetPCLKSEL (uint32_t ClkType);
void UART_ConfigStructInit(UART_CFG_Type *UART_InitStruct);

void UART_Init(LPC_UART_TypeDef *UARTx, UART_CFG_Type *UART_ConfigStruct);
void UART_ConfigStructInit(UART_CFG_Type *UART_InitStruct);


//---------------------------------------------------------------------------------------------
// GPIO related definitions  (see also nrk_gpio_raw functions for platform specific access)
// these macros provide simplified gpio access for
// higher level programs. Pins are defined has NRK_<pin name>;
// mapping to the hardware is done in ulib.c
int8_t nrk_gpio_set(uint32_t pin);
int8_t nrk_gpio_clr(uint32_t pin);
int8_t nrk_gpio_get(uint8_t pin);
int8_t nrk_gpio_toggle(uint32_t pin);
int8_t nrk_gpio_direction(uint32_t pin, uint8_t pin_direction);

int8_t nrk_gpio_pullups( uint8_t enable );

//---------------------------------------------------------------------------------------------
// LED related definitions
int8_t nrk_led_set(int led);
int8_t nrk_led_clr(int led);
int8_t nrk_led_toggle(int led);

//---------------------------------------------------------------------------------------------
// Button related definitions
int8_t nrk_get_button(uint8_t b);



#endif
