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
*  Zane Starr
*******************************************************************************/

#include <include.h>
#include <ulib.h>
#include <stdio.h>
#include <hal.h>
#include <hal_mbed.h>
//#include <avr/interrupt.h>
#include <nrk_pin_define.h>
#include <nrk_error.h>
#include <nrk_events.h>

#ifdef NANORK
#include <nrk_cfg.h>
#endif

#ifdef NRK_UART_BUF
#include <nrk_events.h>

#ifndef MAX_RX_UART_BUF
#define MAX_RX_UART_BUF    16
#endif


/* Private Functions ---------------------------------------------------------- */

static uint8_t uart_set_divisors(LPC_UART_TypeDef *UARTx, uint32_t baudrate);
static LPC_GPIO_TypeDef *GPIO_GetPointer(uint8_t portNum);
static uint8_t uart_rx_buf_start,uart_rx_buf_end;
static char uart_rx_buf[MAX_RX_UART_BUF];
static nrk_sig_t uart_rx_signal; // uint_8 type

//AVR Specific....change this..

/*SIGNAL(USART1_RX_vect)*/ //this is interrupt routine for USART...
extern "C" void UART3_IRQHandler(void)
{
    char c;
    uint8_t sig;
    //cli();
    DISABLE_UART3_RX_INT();
    UART3_WAIT_AND_RECEIVE(c);
    uart_rx_buf[uart_rx_buf_end]=c;
    //if(uart_rx_buf_end==uart_rx_buf_start) sig=1; else sig=0;
    uart_rx_buf_end++;
    if(uart_rx_buf_end==MAX_RX_UART_BUF) uart_rx_buf_end=0;
    nrk_event_signal(uart_rx_signal);
    CLEAR_UART3_RX_INT();
    ENABLE_UART3_RX_INT();
    //sei();
}

char getc3()
{
    char tmp;
    
    if(uart_rx_buf_start!=uart_rx_buf_end)
    {
        tmp = uart_rx_buf[uart_rx_buf_start];
        uart_rx_buf_start++;
        if(uart_rx_buf_start==MAX_RX_UART_BUF) uart_rx_buf_start=0;
        return(tmp);
    }
    // if buffer empty, then we have to block for it
    UART3_WAIT_AND_RECEIVE(tmp);
    return tmp;
}

uint8_t nrk_uart_data_ready(uint8_t uart_num) //check uart 1 and 0 difference
{
    if(uart_num==2)
    {
        if( (LPC_UART2->LSR & ((uint8_t)(1<<0)))  ) return 1;
    }
    if(uart_num==3)
    {
        if(uart_rx_buf_start!=uart_rx_buf_end) return 1;
    }
    return 0;
}

nrk_sig_t nrk_uart_rx_signal_get()
{
   if(uart_rx_signal==NRK_ERROR) nrk_error_add(NRK_SIGNAL_CREATE_ERROR);
   return uart_rx_signal;
}

#else

nrk_sig_t nrk_uart_rx_signal_get()
{
   return NRK_ERROR;
}


uint8_t nrk_uart_data_ready(uint8_t uart_num)
{
    if(uart_num==3)
    {
        if( (LPC_UART3->LSR & ((uint8_t)(1<<0))) ) return 1;
    }
    if(uart_num==2)
    {
        if( (LPC_UART2->LSR & ((uint8_t)(1<<0)))  ) return 1;
    }
    return 0;

}

char getc3(void){
    unsigned char tmp;
    UART3_WAIT_AND_RECEIVE(tmp);
    return tmp;
}

#endif

void nrk_kprintf( const char *addr)
{
    /* Dalton Banks
    char c;
    while((c=pgm_read_byte(addr++)))
        putchar(c);
    */
}

void nrk_setup_ports()
{
    PORT_INIT(); //TODO: check this... Abhijeet
    //SPI_INIT(); AVR specific
}
//Code here is taken from LPC17xx specifc libraries ...Abhijeet
//---------------------------------------------------------------------------------------------
// GPIO related definitions
//---------------------------------------------------------------------------------------------
// Define high-level nrk pins mappings to hardware pins and ports
// This is used for nrk_gpio_... functions.
// Raw GPIO mapping can be found in the nrk_pin_define.h file.
//---------------------------------------------------------------------------------------------

/*
 //Leds
 
 NRK_PIN( LED_0,LED_0, NRK_PORTB )
 NRK_PIN( LED_1,LED_1, NRK_PORTB )
 NRK_PIN( LED_2,LED_2, NRK_PORTB )
 NRK_PIN( LED_3,LED_3, NRK_PORTB )
 
 
 //-------------------------------
 // Port A
 NRK_PIN( DEBUG_0,DEBUG_0, NRK_PORTA )
 NRK_PIN( DEBUG_1,DEBUG_1, NRK_PORTA )
 NRK_PIN( BUTTON,BUTTON, NRK_PORTA )
 //-------------------------------
 // Port B
 NRK_PIN( SPI_SS,SPI_SS, NRK_PORTB )
 NRK_PIN( SCK,SCK, NRK_PORTB )
 NRK_PIN( MOSI,MOSI, NRK_PORTB )
 NRK_PIN( MISO,MISO, NRK_PORTB )
 NRK_PIN( GPIO26,4, NRK_PORTB )
 NRK_PIN( MMC_11,5, NRK_PORTB )
 NRK_PIN( MMC_10,6, NRK_PORTB )
 NRK_PIN( MMC_9,7, NRK_PORTB )
 
 //-------------------------------
 // Port D
 NRK_PIN( DEBUG_2,DEBUG_2, NRK_PORTD )
 NRK_PIN( DEBUG_3,DEBUG_3, NRK_PORTD )
 NRK_PIN( UART1_RXD,UART1_RXD, NRK_PORTD )
 NRK_PIN( UART1_TXD,UART1_TXD, NRK_PORTD )
 NRK_PIN( CCA,CCA, NRK_PORTD )
 NRK_PIN( SFD,SFD, NRK_PORTD )
 
 //-------------------------------
 // Port E
 NRK_PIN( UART0_RXD,UART0_RXD, NRK_PORTE )
 NRK_PIN( UART0_TXD,UART0_TXD, NRK_PORTE )
 
 NRK_PIN( GPIO34,GPIO34, NRK_PORTE )
 NRK_PIN( FIFOP, FIFOP, NRK_PORTE )
 
 
 //-------------------------------
 // Port F
 NRK_PIN( ADC_INPUT_0, ADC_INPUT_0, NRK_PORTF )
 NRK_PIN( ADC_INPUT_1, ADC_INPUT_1, NRK_PORTF )
 NRK_PIN( ADC_INPUT_2, ADC_INPUT_2, NRK_PORTF )
 NRK_PIN( ADC_INPUT_3, ADC_INPUT_3, NRK_PORTF )
 NRK_PIN( ADC_INPUT_4, ADC_INPUT_4, NRK_PORTF )
 NRK_PIN( ADC_INPUT_5, ADC_INPUT_5, NRK_PORTF )
 NRK_PIN( ADC_INPUT_6, ADC_INPUT_6, NRK_PORTF )
 NRK_PIN( ADC_INPUT_7, ADC_INPUT_7, NRK_PORTF )
 */

void PORT_INIT(void) 
{
    initLED(); //led initialisation Abhijeet
    setupSPIPins();
    setup_uartPins();
} 


//Initialise UART Pins on Port 0 for UART 3

void setup_uartPins(void)
{
    PINSEL_CFG_Type pinConfig;
    
	/* Configure pins */
	pinConfig.OpenDrain = PINSEL_PINMODE_NORMAL;
	pinConfig.Pinmode = PINSEL_PINMODE_PULLUP;
	pinConfig.Funcnum = PINSEL_FUNC_2;
	pinConfig.Portnum = NRK_PORTA;
	pinConfig.Pinnum = UART3_RXD;		// UART3 Recieve
	PINSEL_ConfigPin(&pinConfig);
	pinConfig.Pinnum = UART3_TXD;		// UART3 Transmit
	PINSEL_ConfigPin(&pinConfig);
    
    pinConfig.Pinnum = UART2_RXD;		// UART2 Recieve
	PINSEL_ConfigPin(&pinConfig);
	pinConfig.Pinnum = UART2_TXD;		// UART2 Transmit
	PINSEL_ConfigPin(&pinConfig);
	
}

//Initialise SPI PINS on Port 0
void setupSPIPins(void)
{
    PINSEL_CFG_Type pinConfig;
    
	/* Configure pins */
	pinConfig.OpenDrain = PINSEL_PINMODE_NORMAL;
	pinConfig.Pinmode = PINSEL_PINMODE_PULLUP;
	pinConfig.Funcnum = PINSEL_FUNC_3;
	pinConfig.Portnum = NRK_PORTA;
	pinConfig.Pinnum = SPI_SS;		// Slave Select
	PINSEL_ConfigPin(&pinConfig);
	pinConfig.Pinnum = MOSI;		// MOSI
	PINSEL_ConfigPin(&pinConfig);
	pinConfig.Pinnum = MISO;		// MISO
	PINSEL_ConfigPin(&pinConfig);
	pinConfig.Pinnum = SCK;		// Clock
	PINSEL_ConfigPin(&pinConfig);    
}
//Initialise LEDs
//LEDs are on pins 18,20,21,23 of Port 1

void initLED(void) 
{
	LPC_SC->PCONP |= 1<<15;
    PINSEL_CFG_Type pinConfig;
    
	/* Configure pins */
	pinConfig.OpenDrain = PINSEL_PINMODE_NORMAL;
	pinConfig.Pinmode = PINSEL_PINMODE_PULLUP;
	pinConfig.Funcnum = PINSEL_FUNC_0;
	pinConfig.Portnum = NRK_PORTB;
	pinConfig.Pinnum = LED_0;		// LED 0
	PINSEL_ConfigPin(&pinConfig);
	pinConfig.Pinnum = LED_1;		// LED 1
	PINSEL_ConfigPin(&pinConfig);
	pinConfig.Pinnum = LED_2;		// LED 2
	PINSEL_ConfigPin(&pinConfig);
	pinConfig.Pinnum = LED_3;		// LED 3
	PINSEL_ConfigPin(&pinConfig);
    
	/* Enable bits corresponding to LEDs as outputs. */
	GPIO_SetDir(1, LED_0, 1);
    GPIO_SetDir(1, LED_1, 1);
    GPIO_SetDir(1, LED_2, 1);
    GPIO_SetDir(1, LED_3, 1);
	/* Turn off all MBED_MBED_LEDs. */
	GPIO_ClearValue(1, LED_0);
	GPIO_ClearValue(1, LED_1);
	GPIO_ClearValue(1, LED_2);
	GPIO_ClearValue(1, LED_3);
}

/*********************************************************************
 //**
 * @brief 		Configure Pin corresponding to specified parameters passed
 * 				in the PinCfg
 * @param[in]	PinCfg	Pointer to a PINSEL_CFG_Type structure
 *                    that contains the configuration information for the
 *                    specified pin.
 * @return 		None
 **********************************************************************/
void PINSEL_ConfigPin(PINSEL_CFG_Type *PinCfg)
{
	set_PinFunc(PinCfg->Portnum, PinCfg->Pinnum, PinCfg->Funcnum);
	set_ResistorMode(PinCfg->Portnum, PinCfg->Pinnum, PinCfg->Pinmode);
	set_OpenDrainMode(PinCfg->Portnum, PinCfg->Pinnum, PinCfg->OpenDrain);
}


void set_PinFunc ( uint8_t portnum, uint8_t pinnum, uint8_t funcnum)
{
	uint32_t pinnum_t = pinnum;
	uint32_t pinselreg_idx = 2 * portnum;
	uint32_t *pPinCon = (uint32_t *)&LPC_PINCON->PINSEL0;
    
	if (pinnum_t >= 16) {
		pinnum_t -= 16;
		pinselreg_idx++;
	}
	*(uint32_t *)(pPinCon + pinselreg_idx) &= ~(0x03UL << (pinnum_t * 2));
	*(uint32_t *)(pPinCon + pinselreg_idx) |= ((uint32_t)funcnum) << (pinnum_t * 2);
}

void set_ResistorMode ( uint8_t portnum, uint8_t pinnum, uint8_t modenum)
{
	uint32_t pinnum_t = pinnum;
	uint32_t pinmodereg_idx = 2 * portnum;
	uint32_t *pPinCon = (uint32_t *)&LPC_PINCON->PINMODE0;
    
	if (pinnum_t >= 16) {
		pinnum_t -= 16;
		pinmodereg_idx++ ;
	}
    
	*(uint32_t *)(pPinCon + pinmodereg_idx) &= ~(0x03UL << (pinnum_t * 2));
	*(uint32_t *)(pPinCon + pinmodereg_idx) |= ((uint32_t)modenum) << (pinnum_t * 2);
}

void set_OpenDrainMode( uint8_t portnum, uint8_t pinnum, uint8_t modenum)
{
	uint32_t *pPinCon = (uint32_t *)&LPC_PINCON->PINMODE_OD0;
    
	if (modenum == PINSEL_PINMODE_OPENDRAIN){
		*(uint32_t *)(pPinCon + portnum) |= (0x01UL << pinnum);
	} else {
		*(uint32_t *)(pPinCon + portnum) &= ~(0x01UL << pinnum);
	}
}

/* GPIO ------------------------------------------------------------------------------ */

/*********************************************************************//**
  * @brief		Get pointer to GPIO peripheral due to GPIO port
  * @param[in]	portNum		Port Number value, should be in range from 0 to 4.
  * @return		Pointer to GPIO peripheral
  **********************************************************************/

static LPC_GPIO_TypeDef *GPIO_GetPointer(uint8_t portNum)
{
	LPC_GPIO_TypeDef *pGPIO = NULL;
    
	switch (portNum) {
        case 0:
            pGPIO = LPC_GPIO0;
            break;
        case 1:
            pGPIO = LPC_GPIO1;
            break;
        case 2:
            pGPIO = LPC_GPIO2;
            break;
        case 3:
            pGPIO = LPC_GPIO3;
            break;
        case 4:
            pGPIO = LPC_GPIO4;
            break;
        default:
            break;
	}
    
	return pGPIO;
}


/*********************************************************************//**
  * @brief		Set Direction for GPIO port.
  * @param[in]	portNum		Port Number value, should be in range from 0 to 4
  * @param[in]	bitValue	Value that contains all bits to set direction,
  * 							in range from 0 to 0xFFFFFFFF.
  * 							example: value 0x5 to set direction for bit 0 and bit 1.
  * @param[in]	dir			Direction value, should be:
  * 							- 0: Input.
  * 							- 1: Output.
  * @return		None
  *
  * Note: All remaining bits that are not activated in bitValue (value '0')
  * will not be effected by this function.
  **********************************************************************/

void GPIO_SetDir(uint8_t portNum, uint32_t bitValue, uint8_t dir)
{
	LPC_GPIO_TypeDef *pGPIO = GPIO_GetPointer(portNum);
    
	if (pGPIO != NULL) {
		// Enable Output
		if (dir) {
			pGPIO->FIODIR |= (1<<bitValue);
		}
		// Enable Input
		else {
			pGPIO->FIODIR &= ~(1<<bitValue);
		}
	}
}



/*********************************************************************//**
   * @brief		Clear Value for bits that have output direction on GPIO port.
   * @param[in]	portNum		Port number value, should be in range from 0 to 4
   * @param[in]	bitValue	Value that contains all bits on GPIO to clear,
   * 							in range from 0 to 0xFFFFFFFF.
   * 							example: value 0x5 to clear bit 0 and bit 1.
   * @return		None
   *
   * Note:
   * - For all bits that has been set as input direction, this function will
   * not effect.
   * - For all remaining bits that are not activated in bitValue (value '0')
   * will not be effected by this function.
   **********************************************************************/

void GPIO_ClearValue(uint8_t portNum, uint32_t bitValue)
{
	LPC_GPIO_TypeDef *pGPIO = GPIO_GetPointer(portNum);
    
	if (pGPIO != NULL) {
		pGPIO->FIOCLR = (1<<bitValue);
	}
}

/*********************************************************************//**
 * @brief		Set Value for bits that have output direction on GPIO port.
 * @param[in]	portNum		Port number value, should be in range from 0 to 4
 * @param[in]	bitValue	Value that contains all bits on GPIO to set,
 * 							in range from 0 to 0xFFFFFFFF.
 * 							example: value 0x5 to set bit 0 and bit 1.
 * @return		None
 *
 * Note:
 * - For all bits that has been set as input direction, this function will
 * not effect.
 * - For all remaining bits that are not activated in bitValue (value '0')
 * will not be effected by this function.
 **********************************************************************/

void GPIO_SetValue(uint8_t portNum, uint32_t bitValue)
{
	LPC_GPIO_TypeDef *pGPIO = GPIO_GetPointer(portNum);
    
	if (pGPIO != NULL) {
		pGPIO->FIOSET = (1<<bitValue);
	}
}

/*********************************************************************//**
  * @brief		Read Current state on port pin that have input direction of GPIO
  * @param[in]	portNum		Port number to read value, in range from 0 to 4
  * @return		Current value of GPIO port.
  *
  * Note: Return value contain state of each port pin (bit) on that GPIO regardless
  * its direction is input or output.
  **********************************************************************/
uint32_t GPIO_ReadValue(uint8_t portNum)
{
	LPC_GPIO_TypeDef *pGPIO = GPIO_GetPointer(portNum);
    
	if (pGPIO != NULL) {
		return pGPIO->FIOPIN;
	}
    
	return (0);
}

/*********************************************************************//**
   * @brief		Toggle Value for bits that have output direction on GPIO port.
   * @param[in]	portNum		Port number value, should be in range from 0 to 4
   * @param[in]	bitValue	Value that contains all bits on GPIO to set,
   * 							in range from 0 to 0xFFFFFFFF.
   * 							example: value 0x5 to set bit 0 and bit 1.
   * @return		None
   
   **********************************************************************/
void GPIO_Toggle(uint8_t portNum, uint32_t bitValue)
{
	LPC_GPIO_TypeDef *pGPIO = GPIO_GetPointer(portNum);
    
	if (pGPIO != NULL) {
		pGPIO->FIOPIN ^= (1<<bitValue);
	}
}

//-------------------------------
// GPIO handling functions


int8_t nrk_gpio_set(uint32_t pin)
{
    if (pin == NRK_INVALID_PIN_VAL) return -1;
    switch (pin) {
        case LED_0:       
        case LED_1:      
        case LED_2:         
        case LED_3:
            GPIO_SetValue(1, pin); break; 
        default: return -1;
    }
    return 1;
}

int8_t nrk_gpio_clr(uint32_t pin)
{
    if (pin == NRK_INVALID_PIN_VAL) return -1;
    switch (pin) {
        case LED_0:       
        case LED_1:      
        case LED_2:         
        case LED_3:
            GPIO_ClearValue(1, pin); break; 
        default: return -1;
    }
    return 1;
}

int8_t nrk_gpio_get(uint8_t pin) //todo
{
    if (pin == NRK_INVALID_PIN_VAL) return -1;
    switch (pin) {
        case LED_0:
        case LED_1:
        case LED_2:
        case LED_3:
            return (uint8_t (GPIO_ReadValue(1) & (1<<pin)));
        default: return -1;
    }
    return 1;
}

int8_t nrk_gpio_toggle(uint32_t pin)
{
    if (pin == NRK_INVALID_PIN_VAL) return -1;
    switch (pin) {
        case LED_0:       
        case LED_1:      
        case LED_2:         
        case LED_3:
            GPIO_Toggle(1, pin); break; 
        default: return -1;
    }
    return 1;
}

int8_t nrk_gpio_direction(uint32_t pin, uint8_t pin_direction)
{
    if (pin == NRK_INVALID_PIN_VAL) return -1;
    
    switch (pin) {
        case LED_0:       
        case LED_1:      
        case LED_2:         
        case LED_3:
            GPIO_SetDir(1, pin, pin_direction); break; 
        default: return -1;
    }
    
    return 1;
}

int8_t nrk_get_button(uint8_t b)
{
    /*
     if(b==0) {
        return( !(PINA & BM(BUTTON))); 
	} 
    return -1;
     */
}

int8_t nrk_led_toggle( int led )
{
    if(led==0) { nrk_gpio_toggle(LED_0); return 1; }
    if(led==1) { nrk_gpio_toggle(LED_1); return 1; }
    if(led==2) { nrk_gpio_toggle(LED_2); return 1; }
    if(led==3) { nrk_gpio_toggle(LED_3); return 1; }
    return -1;
}

int8_t nrk_led_set( int led )
{
    if(led==0) { nrk_gpio_set(LED_0); return 1; }
    if(led==1) { nrk_gpio_set(LED_1); return 1; }
    if(led==2) { nrk_gpio_set(LED_2); return 1; }
    if(led==3) { nrk_gpio_set(LED_3); return 1; }
    return -1;
}

int8_t nrk_led_clr( int led )
{
    if(led==0) { nrk_gpio_clr(LED_0); return 1; }
    if(led==1) { nrk_gpio_clr(LED_1); return 1; }
    if(led==2) { nrk_gpio_clr(LED_2); return 1; }
    if(led==3) { nrk_gpio_clr(LED_3); return 1; }
    return -1;
}

int8_t nrk_gpio_pullups(uint8_t enable)
{
    /*
     if(enable) MCUCR &= ~BM(PUD);
     else MCUCR |= BM(PUD);
     */
     return NRK_OK;
     
}


/*
 void IO_SET_E(uint8_t pin)
 {
 PORTE |= BM(pin);
 }
 
 void IO_CLR_E(uint8_t pin)
 {
 PORTE &= ~BM(pin);
 }
 
 void IO_SET_F(uint8_t pin)
 {
 PORTF |= BM(pin);
 }
 
 void IO_CLR_F(uint8_t pin)
 {
 PORTF &= ~BM(pin);
 }
 */


//Abhijeet

void putc2(char x)
{
    UART2_WAIT_AND_SEND(x);
}

void putc3(char x)
{
    UART3_WAIT_AND_SEND(x);
}

void setup_uart3(uint32_t baudrate)
{    
    UART_CFG_Type uartConfig;
    
    /* Initialize UART. */
	UART_ConfigStructInit(&uartConfig);
	uartConfig.Baud_rate = baudrate;
	uartConfig.Parity = UART_PARITY_NONE;
	uartConfig.Databits = UART_DATABIT_8;
	uartConfig.Stopbits = UART_STOPBIT_1;
	UART_Init(LPC_UART3, &uartConfig);
    ENABLE_UART3();
}

void setup_uart2(uint16_t baudrate)
{
    UART_CFG_Type uartConfig;
    
    /* Initialize UART. */
	UART_ConfigStructInit(&uartConfig);
	uartConfig.Baud_rate = baudrate;
	uartConfig.Parity = UART_PARITY_NONE;
	uartConfig.Databits = UART_DATABIT_8;
	uartConfig.Stopbits = UART_STOPBIT_1;
	UART_Init(LPC_UART2, &uartConfig);
    ENABLE_UART2();
}

uint32_t CLKPWR_GetPCLK (uint32_t ClkType)
{
	uint32_t retval, div;
    
	retval = SystemCoreClock;
	div = CLKPWR_GetPCLKSEL(ClkType);
    
	switch (div)
	{
        case 0:
            div = 4;
            break;
            
        case 1:
            div = 1;
            break;
            
        case 2:
            div = 2;
            break;
            
        case 3:
            div = 8;
            break;
	}
	retval /= div;
    
	return retval;
}

uint32_t CLKPWR_GetPCLKSEL (uint32_t ClkType)
{
	uint32_t bitpos, retval;
    
	if (ClkType < 32)
	{
		bitpos = ClkType;
		retval = LPC_SC->PCLKSEL0;
	}
	else
	{
		bitpos = ClkType - 32;
		retval = LPC_SC->PCLKSEL1;
	}
    
	retval = CLKPWR_PCLKSEL_GET(bitpos, retval);
	return retval;
}

/*********************************************************************//**
  * @brief		Determines best dividers to get a target clock rate
  * @param[in]	UARTx	Pointer to selected UART peripheral, should be:
  * 				- LPC_UART0: UART0 peripheral
  * 				- LPC_UART1: UART1 peripheral
  * 				- LPC_UART2: UART2 peripheral
  * 				- LPC_UART3: UART3 peripheral
  * @param[in]	baudrate Desired UART baud rate.
  * @return 		Error status, could be:
  * 				- SUCCESS
  * 				- ERROR
  **********************************************************************/

uint8_t uart_set_divisors(LPC_UART_TypeDef *UARTx, uint32_t baudrate)
{
	uint8_t errorStatus = 0;
    
	uint32_t uClk;
	uint32_t d, m, bestd, bestm, tmp;
	uint64_t best_divisor, divisor;
	uint32_t current_error, best_error;
	uint32_t recalcbaud;
    
	/* get UART block clock */
	if (UARTx == (LPC_UART_TypeDef *)LPC_UART0)
	{
		uClk = CLKPWR_GetPCLK (CLKPWR_PCLKSEL_UART0);
	}
	else if (UARTx == (LPC_UART_TypeDef *)LPC_UART1)
	{
		uClk = CLKPWR_GetPCLK (CLKPWR_PCLKSEL_UART1);
	}
	else if (UARTx == LPC_UART2)
	{
		uClk = CLKPWR_GetPCLK (CLKPWR_PCLKSEL_UART2);
	}
	else if (UARTx == LPC_UART3)
	{
		uClk = CLKPWR_GetPCLK (CLKPWR_PCLKSEL_UART3);
	}
    
    
	/* In the Uart IP block, baud rate is calculated using FDR and DLL-DLM registers
     * The formula is :
     * BaudRate= uClk * (mulFracDiv/(mulFracDiv+dividerAddFracDiv) / (16 * (DLL)
     * It involves floating point calculations. That's the reason the formulae are adjusted with
     * Multiply and divide method.*/
	/* The value of mulFracDiv and dividerAddFracDiv should comply to the following expressions:
     * 0 < mulFracDiv <= 15, 0 <= dividerAddFracDiv <= 15 */
	best_error = 0xFFFFFFFF; /* Worst case */
	bestd = 0;
	bestm = 0;
	best_divisor = 0;
	for (m = 1 ; m <= 15 ;m++)
	{
		for (d = 0 ; d < m ; d++)
		{
            divisor = ((uint64_t)uClk<<28)*m/(baudrate*(m+d));
            current_error = divisor & 0xFFFFFFFF;
            
            tmp = divisor>>32;
            
            /* Adjust error */
            if(current_error > ((uint32_t)1<<31)){
                current_error = -current_error;
                tmp++;
			}
            
            if(tmp<1 || tmp>65536) /* Out of range */
                continue;
            
            if( current_error < best_error){
                best_error = current_error;
                best_divisor = tmp;
                bestd = d;
                bestm = m;
                if(best_error == 0) break;
			}
		} /* end of inner for loop */
        
		if (best_error == 0)
            break;
	} /* end of outer for loop  */
    
	if(best_divisor == 0) return 0; /* can not find best match */
    
	recalcbaud = (uClk>>4) * bestm/(best_divisor * (bestm + bestd));
    
	/* reuse best_error to evaluate baud error*/
	if(baudrate>recalcbaud) best_error = baudrate - recalcbaud;
	else best_error = recalcbaud -baudrate;
    
	best_error = best_error * 100 / baudrate;
    
	if (best_error < UART_ACCEPTED_BAUDRATE_ERROR)
    {
        if (((LPC_UART1_TypeDef *)UARTx) == LPC_UART1)
        {
            ((LPC_UART1_TypeDef *)UARTx)->LCR |= ((uint8_t)(1<<7)) ;
            ((LPC_UART1_TypeDef *)UARTx)->/*DLIER.*/DLM = UART_LOAD_DLM(best_divisor);
            ((LPC_UART1_TypeDef *)UARTx)->/*RBTHDLR.*/DLL = UART_LOAD_DLL(best_divisor);
            /* Then reset DLAB bit */
            ((LPC_UART1_TypeDef *)UARTx)->LCR &= (~((uint8_t)(1<<7))) & ((uint8_t)(0xFF));
            ((LPC_UART1_TypeDef *)UARTx)->FDR = (UART_FDR_MULVAL(bestm) \
                                                 | UART_FDR_DIVADDVAL(bestd)) & UART_FDR_BITMASK;
        }
        else
        {
            UARTx->LCR |= ((uint8_t)(1<<7));
            UARTx->/*DLIER.*/DLM = UART_LOAD_DLM(best_divisor);
            UARTx->/*RBTHDLR.*/DLL = UART_LOAD_DLL(best_divisor);
            /* Then reset DLAB bit */
            UARTx->LCR &= (~UART_LCR_DLAB_EN) & UART_LCR_BITMASK;
            UARTx->FDR = (UART_FDR_MULVAL(bestm) \
                          | UART_FDR_DIVADDVAL(bestd)) & UART_FDR_BITMASK;
        }
        errorStatus = 1;
    }
    
    return errorStatus;
}


/* UART Init functions -------------------------------------------------*/
/********************************************************************//**
  * @brief		Initializes the UARTx peripheral according to the specified
  *               parameters in the UART_ConfigStruct.
  * @param[in]	UARTx	UART peripheral selected, should be:
  *   			- LPC_UART0: UART0 peripheral
  * 				- LPC_UART1: UART1 peripheral
  * 				- LPC_UART2: UART2 peripheral
  * 				- LPC_UART3: UART3 peripheral
  * @param[in]	UART_ConfigStruct Pointer to a UART_CFG_Type structure
  *                    that contains the configuration information for the
  *                    specified UART peripheral.
  * @return 		None
  *********************************************************************/

void UART_Init(LPC_UART_TypeDef *UARTx, UART_CFG_Type *UART_ConfigStruct)
{
	uint32_t tmp;
    
	if(UARTx == LPC_UART3)
	{
		/* Set up clock and power for UART module */
        LPC_SC->PCONP |= (1<<25) & CLKPWR_PCONP_BITMASK;
	}
    if(UARTx == LPC_UART2)
	{
		/* Set up clock and power for UART module */
        LPC_SC->PCONP |= (1<<24) & CLKPWR_PCONP_BITMASK;
	}
    /* FIFOs are empty */
    //UARTx->/*IIFCR.*/FCR = ( UART_FCR_FIFO_EN | UART_FCR_RX_RS | UART_FCR_TX_RS);
    // Disable FIFO
    //UARTx->/*IIFCR.*/FCR = 0;
    
    // Dummy reading
    while (UARTx->LSR & (1<<0))
    {
        tmp = UARTx->/*RBTHDLR.*/RBR;
    }
    
    UARTx->TER = ((uint8_t)(1<<7)); //transmit enable
    // Wait for current transmit complete
    while (!(UARTx->LSR & ((uint8_t)(1<<5)))); 
    // Disable Tx
    UARTx->TER = 0;
    
    // Disable interrupt
    UARTx->/*DLIER.*/IER = 0;
    // Set LCR to default state
    UARTx->LCR = 0;
    // Set ACR to default state
    UARTx->ACR = 0;
    // Dummy reading
    tmp = UARTx->LSR;
    
    
	if (UARTx == LPC_UART3)
	{
		// Set IrDA to default state
		UARTx->ICR = 0;
	}
    
	// Set Line Control register ----------------------------
    
	uart_set_divisors(UARTx, (UART_ConfigStruct->Baud_rate));
    
	if (((LPC_UART1_TypeDef *)UARTx) == LPC_UART1)
	{
		//tmp = (((LPC_UART1_TypeDef *)UARTx)->LCR & (((uint8_t)(1<<7))   | ((uint8_t)(1<<6))	)) \& ((uint8_t)(0xFF))	;
	}
	else
	{
		tmp = (UARTx->LCR & ( (((uint8_t)(1<<7)) | ((uint8_t)(1<<6)))) & ((uint8_t)(0xFF)));
    }
               
               switch (UART_ConfigStruct->Databits){
                   case UART_DATABIT_5:
                       tmp |= UART_LCR_WLEN5;
                       break;
                   case UART_DATABIT_6:
                       tmp |= UART_LCR_WLEN6;
                       break;
                   case UART_DATABIT_7:
                       tmp |= UART_LCR_WLEN7;
                       break;
                   case UART_DATABIT_8:
                   default:
                       tmp |= UART_LCR_WLEN8;
                       break;
               }
               
               if (UART_ConfigStruct->Parity == UART_PARITY_NONE)
               {
                   // Do nothing...
               }
               else
               {
                   tmp |= UART_LCR_PARITY_EN;
                   switch (UART_ConfigStruct->Parity)
                   {
                       case UART_PARITY_ODD:
                           tmp |= UART_LCR_PARITY_ODD;
                           break;
                           
                       case UART_PARITY_EVEN:
                           tmp |= UART_LCR_PARITY_EVEN;
                           break;
                           
                       case UART_PARITY_SP_1:
                           tmp |= UART_LCR_PARITY_F_1;
                           break;
                           
                       case UART_PARITY_SP_0:
                           tmp |= UART_LCR_PARITY_F_0;
                           break;
                       default:
                           break;
                   }
               }
               
               switch (UART_ConfigStruct->Stopbits){
                   case UART_STOPBIT_2:
                       tmp |= UART_LCR_STOPBIT_SEL;
                       break;
                   case UART_STOPBIT_1:
                   default:
                       // Do no thing
                       break;
               }
               
               
               // Write back to LCR, configure FIFO and Disable Tx
               if (((LPC_UART1_TypeDef *)UARTx) ==  LPC_UART1)
               {
                   ((LPC_UART1_TypeDef *)UARTx)->LCR = (uint8_t)(tmp & UART_LCR_BITMASK);
               }
               else
               {
                   UARTx->LCR = (uint8_t)(tmp & UART_LCR_BITMASK);
               }
               }
               
               void UART_ConfigStructInit(UART_CFG_Type *UART_InitStruct)
        {
            UART_InitStruct->Baud_rate = 9600;
            UART_InitStruct->Databits = UART_DATABIT_8;
            UART_InitStruct->Parity = UART_PARITY_NONE;
            UART_InitStruct->Stopbits = UART_STOPBIT_1;
        }
               
/**
 * nrk_setup_uart()
 *
 * Sets a default uart for a given platform and
 * direct stdin and stdout to that port.
 *
 * More advanced UART usage will require manually
 * setting parameters.
 */
void nrk_setup_uart(uint32_t baudrate)
{
    
    setup_uart3(baudrate);
    //setup_uart2(baudrate);
    
    //stdout = fdevopen( (void *)putc3, (void *)getc3);
    //stdin = fdevopen( (void *)putc3, (void *)getc3);
            
    #ifdef NRK_UART_BUF
            uart_rx_signal=nrk_signal_create();
            if(uart_rx_signal==NRK_ERROR) nrk_error_add(NRK_SIGNAL_CREATE_ERROR);
            uart_rx_buf_start=0;
            uart_rx_buf_end=0;
            ENABLE_UART3_RX_INT();
            NVIC_EnableIRQ(UART3_IRQn);
    #endif
            
}
               
               
               
/* get one char from uart */
char getc2(void){
    unsigned char tmp;
    UART2_WAIT_AND_RECEIVE(tmp);
    return tmp;
}
