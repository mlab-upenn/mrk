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
 *  Nuno Pereira
 *  Anthony Rowe
 *******************************************************************************/


#ifndef NRK_PIN_DEFINE_H
#define NRK_PIN_DEFINE_H

/***********************************************************************
 * Macros define for Pin Function selection
 **********************************************************************/
#define PINSEL_FUNC_0	((0))	/**< default function*/
#define PINSEL_FUNC_1	((1))	/**< first alternate function*/
#define PINSEL_FUNC_2	((2))	/**< second alternate function*/
#define PINSEL_FUNC_3	((3))	/**< third or reserved alternate function*/

/***********************************************************************
 * Macros define for Pin mode
 **********************************************************************/
#define PINSEL_PINMODE_PULLUP		((0))	/**< Internal pull-up resistor*/
#define PINSEL_PINMODE_TRISTATE 	((2))	/**< Tri-state */
#define PINSEL_PINMODE_PULLDOWN 	((3)) 	/**< Internal pull-down resistor */

/***********************************************************************
 * Macros define for Pin mode (normal/open drain)
 **********************************************************************/
#define	PINSEL_PINMODE_NORMAL		((0))	/**< Pin is in the normal (not open drain) mode.*/
#define	PINSEL_PINMODE_OPENDRAIN	((1)) 	/**< Pin is in the open drain mode */

/* Public Types --------------------------------------------------------------- */
/** @defgroup PINSEL_Public_Types PINSEL Public Types
 * @{
 */

/** @brief Pin configuration structure */
typedef struct
{
	uint8_t Portnum;	/**< Port Number, should be PINSEL_PORT_x,
                         where x should be in range from 0 to 4 */
	uint8_t Pinnum;		/**< Pin Number, should be PINSEL_PIN_x,
                         where x should be in range from 0 to 31 */
	uint8_t Funcnum;	/**< Function Number, should be PINSEL_FUNC_x,
                         where x should be in range from 0 to 3 */
	uint8_t Pinmode;	/**< Pin Mode, should be:
                         - PINSEL_PINMODE_PULLUP: Internal pull-up resistor
                         - PINSEL_PINMODE_TRISTATE: Tri-state
                         - PINSEL_PINMODE_PULLDOWN: Internal pull-down resistor */
	uint8_t OpenDrain;	/**< OpenDrain mode, should be:
                         - PINSEL_PINMODE_NORMAL: Pin is in the normal (not open drain) mode
                         - PINSEL_PINMODE_OPENDRAIN: Pin is in the open drain mode */
} PINSEL_CFG_Type;

/********************************************************************//**
                                                                       * @brief UART Configuration Structure definition
                                                                       **********************************************************************/

/*******************************************************************************************************
 *******************************************************************************************************
 **************************                        UART                       **************************
 *******************************************************************************************************
 *******************************************************************************************************/

/* Accepted Error baud rate value (in percent unit) */
#define UART_ACCEPTED_BAUDRATE_ERROR	(3)			/*!< Acceptable UART baudrate error */
#define UART_LOAD_DLM(div)  (((div) >> 8) & 0xFF)	/**< Macro for loading most significant halfs of divisors */
#define UART_LOAD_DLL(div)	((div) & 0xFF)	/**< Macro for loading least significant halfs of divisors */



/***********************************************************************
 * Macros define for UART
 **********************************************************************/
#define CLKPWR_PCONP_BITMASK	0xEFEFF7DE
/** Peripheral clock divider bit position for UART2 */
#define	CLKPWR_PCLKSEL_UART2  		((uint32_t)(48))
/** Peripheral clock divider bit position for UART3 */
#define	CLKPWR_PCLKSEL_UART3  		((uint32_t)(50))
/** Peripheral clock divider bit position for UART0 */
#define	CLKPWR_PCLKSEL_UART0  		((uint32_t)(6))
/** Peripheral clock divider bit position for UART1 */
#define	CLKPWR_PCLKSEL_UART1  		((uint32_t)(8))

#define CLKPWR_PCLKSEL_GET(p, n)	((uint32_t)((n>>p)&0x03))


/*********************************************************************//**
                                                                        * Macro defines for Macro defines for UART Fractional divider register
                                                                        **********************************************************************/

#define UART_FDR_DIVADDVAL(n)	((uint32_t)(n&0x0F))		/**< Baud-rate generation pre-scaler divisor */
#define UART_FDR_MULVAL(n)		((uint32_t)((n<<4)&0xF0))	/**< Baud-rate pre-scaler multiplier value */
#define UART_FDR_BITMASK		((uint32_t)(0xFF))			/**< UART Fractional Divider register bit mask */

/*********************************************************************//**
                                                                        * Macro defines for Macro defines for UART line control register
                                                                        **********************************************************************/
#define UART_LCR_WLEN5     		((uint8_t)(0))   		/*!< UART 5 bit data mode */
#define UART_LCR_WLEN6     		((uint8_t)(1<<0))   	/*!< UART 6 bit data mode */
#define UART_LCR_WLEN7     		((uint8_t)(2<<0))   	/*!< UART 7 bit data mode */
#define UART_LCR_WLEN8     		((uint8_t)(3<<0))   	/*!< UART 8 bit data mode */
#define UART_LCR_STOPBIT_SEL	((uint8_t)(1<<2))   	/*!< UART Two Stop Bits Select */
#define UART_LCR_PARITY_EN		((uint8_t)(1<<3))		/*!< UART Parity Enable */
#define UART_LCR_PARITY_ODD		((uint8_t)(0))         	/*!< UART Odd Parity Select */
#define UART_LCR_PARITY_EVEN	((uint8_t)(1<<4))		/*!< UART Even Parity Select */
#define UART_LCR_PARITY_F_1		((uint8_t)(2<<4))		/*!< UART force 1 stick parity */
#define UART_LCR_PARITY_F_0		((uint8_t)(3<<4))		/*!< UART force 0 stick parity */
#define UART_LCR_BREAK_EN		((uint8_t)(1<<6))		/*!< UART Transmission Break enable */
#define UART_LCR_DLAB_EN		((uint8_t)(1<<7))    	/*!< UART Divisor Latches Access bit enable */
#define UART_LCR_BITMASK		((uint8_t)(0xFF))		/*!< UART line control bit mask */


/**
 * @brief UART Databit type definitions
 */
typedef enum {
	UART_DATABIT_5		= 0,     		/*!< UART 5 bit data mode */
	UART_DATABIT_6,		     			/*!< UART 6 bit data mode */
	UART_DATABIT_7,		     			/*!< UART 7 bit data mode */
	UART_DATABIT_8		     			/*!< UART 8 bit data mode */
} UART_DATABIT_Type;

/**
 * @brief UART Stop bit type definitions
 */
typedef enum {
	UART_STOPBIT_1		= (0),   					/*!< UART 1 Stop Bits Select */
	UART_STOPBIT_2		 							/*!< UART Two Stop Bits Select */
} UART_STOPBIT_Type;

/**
 * @brief UART Parity type definitions
 */
typedef enum {
	UART_PARITY_NONE 	= 0,					/*!< No parity */
	UART_PARITY_ODD,	 						/*!< Odd parity */
	UART_PARITY_EVEN, 							/*!< Even parity */
	UART_PARITY_SP_1, 							/*!< Forced "1" stick parity */
	UART_PARITY_SP_0 							/*!< Forced "0" stick parity */
} UART_PARITY_Type;


typedef struct {
    uint32_t Baud_rate;   		/**< UART baud rate */
    UART_PARITY_Type Parity;    	/**< Parity selection, should be:
                                     - UART_PARITY_NONE: No parity
                                     - UART_PARITY_ODD: Odd parity
                                     - UART_PARITY_EVEN: Even parity
                                     - UART_PARITY_SP_1: Forced "1" stick parity
                                     - UART_PARITY_SP_0: Forced "0" stick parity
                                     */
    UART_DATABIT_Type Databits;   /**< Number of data bits, should be:
                                   - UART_DATABIT_5: UART 5 bit data mode
                                   - UART_DATABIT_6: UART 6 bit data mode
                                   - UART_DATABIT_7: UART 7 bit data mode
                                   - UART_DATABIT_8: UART 8 bit data mode
                                   */
    UART_STOPBIT_Type Stopbits;   /**< Number of stop bits, should be:
                                   - UART_STOPBIT_1: UART 1 Stop Bits Select
                                   - UART_STOPBIT_2: UART 2 Stop Bits Select
                                   */
} UART_CFG_Type;




/**
 * @brief Functional State Definition
 */
typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;

/**
 * @}
 */
/*******************************************************************************************************
 *******************************************************************************************************
 **************************                        LEDS                       **************************
 *******************************************************************************************************
 *******************************************************************************************************/


//---------------------------------------------------------------------------------------------
//Led Pins on Port 1

#define LED_0	18  // P1.18 LED_0
#define LED_1	20  // P1.20 LED_1   
#define LED_2	21  // P1.21 LED_2
#define LED_3	23  // P1.23 LED_3   

//---------------------------------------------------------------------------------------------
//  SPI
#define SPI_SS          16  // P0.16 - Output: SPI Slave Select
#define SCK             15  // P0.15 - Output: SPI Serial Clock (SCLK)
#define MOSI            18  // P0.18 - Output: SPI Master out - slave in (MOSI)
#define MISO            17  // P0.17 - Input:  SPI Master in - slave out (MISO)

//---------------------------------------------------------------------------------------------
// USART  (3 instead of 1)
#define UART3_RXD       1 // P0.1 - Input:  UART1 RXD  //DIP 9
#define UART3_TXD       0 // P0.0 - Output: UART1 TXD  //DIP 10

#define UART2_RXD       11 // P0.11 - Input:  UART2 RXD //DIP 27
#define UART2_TXD       10 // P0.10 - Output: UART2 TXD //DIP 28

#define UART1_RTS       5 // PD.5 - Output: UART HW handshaking: RTS
#define UART1_CTS       7 // PD.7 - Input:  UART HW handshaking: CTS



/*******************************************************************************************************
 *******************************************************************************************************
 **************************                        GPIO                       **************************
 *******************************************************************************************************
 *******************************************************************************************************/


//---------------------------------------------------------------------------------------------
// Port A
#define VREG_EN         5  // PA.5 - Output: VREG_EN to CC2420
#define RESET_N         6  // PA.6 - Output: RESET_N to CC2420
#define DEBUG_0         3
#define DEBUG_1         4
#define BUTTON          7  // PA.7 - Input button 0

//---------------------------------------------------------------------------------------------
//  PORT B
#define GPIO26		4
#define MMC_11		5
#define MMC_10		6
#define MMC_9		7

//---------------------------------------------------------------------------------------------
//PORT C
#define CSN             0  // PB.0 - Output: SPI Chip Select (CS_N)
#define FIFO            1  // PB.7 - Input:  FIFO from CC2420



//----------------------------------------------------------------------------------------------
// Port D
#define DEBUG_2		0 // PD.0 - Output: I2C_SCL  (USE as DEBUG_2) 
#define DEBUG_3		1 // PD.1 - Output: I2C_SDA  (USE as DEBUG_3) 

#define CCA             4 // PD.6 - Input:  CCA from CC2420

#define SFD             6 // PD.4 - Input:  SFD from CC2420




//----------------------------------------------------------------------------------------------
// Port E


#define GPIO34		6
#define FIFOP           7 // PE.7 - Input:  FIFOP from CC2420
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
// Port F
#define ADC_INPUT_0     0
#define ADC_INPUT_1     1 // PF.1 - ADC1
#define ADC_INPUT_2     2 // PF.2 - ADC2
#define ADC_INPUT_3     3 // PF.3 - ADC3
#define ADC_INPUT_4     4 // PF.3 - ADC3
#define ADC_INPUT_5     5 // PF.3 - ADC3
#define ADC_INPUT_6     6 // PF.3 - ADC3
#define ADC_INPUT_7     7 // PF.3 - ADC3

//-------------------------------------------------------------------------------------------------------
// External RAM interface:
//     PA and PC - Multiplexed address/data
//     PG.0 - Output: Write enable: WR_N
//     PG.1 - Output: Read enable: RD_N
//     PG.2 - Output: Address Latch Enable: ALE
//-------------------------------------------------------------------------------------------------------



//-------------------------------
// GPIO handling functions
// these macros perform raw hw access
// ports and pins are acctual hw ports and pins
/*
 // use pin_port, and pin; ie: nkr_gpio_raw_set( PORTB, DEBUG_0 )
 #define nrk_gpio_raw_set( _port, _pin ) {do { _port |= BM(_pin); } while(0);}
 // use pin_port, and pin; ie: nkr_gpio_raw_clr( PORTB, DEBUG_0 )
 #define nrk_gpio_raw_clr( _port, _pin ) {do { _port &= ~BM(_pin); } while(0);}
 // use pin_port, and pin; ie: nkr_gpio_raw_get( PINB, DEBUG_0 )
 #define nrk_gpio_raw_get( _pin_port, _pin ) (_pin_port & BM(_pin))
 // use pin_port, port and pin; ie: nkr_gpio_raw_toggle( PINB, PORTB, DEBUG_0 )
 #define nrk_gpio_raw_toggle( _pin_port, _port, _pin ) { \
 if ((_pin_port & BM(_pin))) do{ _port &= ~BM(_pin); } while(0); \
 else do { _port |= BM(_pin); }while(0);  \
 }
 // use direction; ie: nkr_gpio_raw_direction( DDRB, DEBUG_0 )
 #define nrk_gpio_raw_direction( _direction_port_name, _pin, _pin_direction ) { \
 if (_pin_direction == NRK_PIN_INPUT) { \
 _direction_port_name &= ~BM( _pin ); \
 } else { \
 _direction_port_name |= BM( _pin ); \
 } \
 }
 
 */

void PINSEL_ConfigPin(PINSEL_CFG_Type *PinCfg);

// when a platform does not support one
// of the NRK_<pin name> declared below, it
// must define it has an invalid pin in the
// platform ulib.c (e.g. a platform that does not
// support NRK_DEBUG_0 should have the following in
// ulib.c NRK_INVALID_PIN( NRK_DEBUG_0 ) )
#define NRK_INVALID_PIN_VAL 0xFF

// nrk ports NRK_<hw port> used for the mapping
// to the real hw. (3 bits reserved for ports)
/*
 #define NRK_PORTA 0
 #define NRK_PORTB 1
 #define NRK_PORTC 2
 #define NRK_PORTD 3
 #define NRK_PORTE 4
 #define NRK_PORTF 5
 */

//4 Ports for mbed

#define NRK_PORTA  NRK_PORT0
#define NRK_PORTB  NRK_PORT1
#define NRK_PORTC  NRK_PORT2
#define NRK_PORTD  NRK_PORT3

#define NRK_PORT0 0
#define NRK_PORT1 1
#define NRK_PORT2 2
#define NRK_PORT3 3

// define pin directions
#define NRK_PIN_INPUT 0
#define NRK_PIN_OUTPUT 1


//---------------------------------------------------------------------------------------------
// GPIO related definitions

// macros to define a pin as used by higher level programs.
// higher level programs refer to pin as NRK_<pin name>
// these functions declare these NRK_<pin name> pins and provide
// the mappings to the hardware
#define DECLARE_NRK_PIN( _pin_name ) extern const uint8_t NRK_ ## _pin_name;
//#define NRK_PIN( _pin_name, _pin , _port ) const uint8_t NRK_ ## _pin_name = (_pin << 3) + (_port & 0x07);
#define NRK_INVALID_PIN( _pin_name ) const uint8_t NRK_ ## _pin_name = NRK_INVALID_PIN_VAL;

// declare pins as used by higher level programs
// mapping to the hardware is done by ulib.c
DECLARE_NRK_PIN( DEBUG_0 ) 			// declare pin named NRK_DEBUG_0
DECLARE_NRK_PIN( DEBUG_1 ) 			// declare pin named NRK_DEBUG_1
DECLARE_NRK_PIN( DEBUG_2 ) 			// (I2C_SCL) 
DECLARE_NRK_PIN( DEBUG_3 ) 			// (I2C_SDA) 
DECLARE_NRK_PIN( DEBUG_1 ) 			// declare pin named NRK_DEBUG_1
DECLARE_NRK_PIN( BUTTON ) 			// declare pin named NRK_BUTTON

DECLARE_NRK_PIN( SPI_SS ) 			// declare pin named NRK_SPI_SS
DECLARE_NRK_PIN( SCK ) 				// declare pin named NRK_SCK
DECLARE_NRK_PIN( MOSI ) 			// declare pin named NRK_MOSI
DECLARE_NRK_PIN( MISO ) 			// declare pin named NRK_MISO

DECLARE_NRK_PIN( MMC_9) 			// declare pin named NRK_GPIO28
DECLARE_NRK_PIN( MMC_10) 			// declare pin named NRK_GPIO28
DECLARE_NRK_PIN( MMC_11) 			// declare pin named NRK_GPIO28
DECLARE_NRK_PIN( GPIO26 ) 			// declare pin named NRK_GPIO26


DECLARE_NRK_PIN( UART3_RXD ) 			// declare pin named NRK_UART1_RXD
DECLARE_NRK_PIN( UART3_TXD ) 			// declare pin named NRK_UART1_TXD
DECLARE_NRK_PIN( SFD ) 				// declare pin named NRK_SFD
DECLARE_NRK_PIN( CCA ) 				// declare pin named NRK_CCA

DECLARE_NRK_PIN( UART2_RXD ) 			// declare pin named NRK_UART0_RXD
DECLARE_NRK_PIN( UART2_TXD ) 			// declare pin named NRK_UART0_TXD
DECLARE_NRK_PIN( FIFOP ) 			// declare pin named NRK_FIFOP
DECLARE_NRK_PIN( LED_0 ) 			// declare pin named NRK_YLED
DECLARE_NRK_PIN( LED_1 ) 			// declare pin named NRK_GLED
DECLARE_NRK_PIN( LED_2 ) 			// declare pin named NRK_RLED
DECLARE_NRK_PIN( LED_3 ) 			// declare pin named NRK_BLED

DECLARE_NRK_PIN( GPIO34 )			// declare pin named NRK_GPIO34

DECLARE_NRK_PIN( ADC_INPUT_0 )
DECLARE_NRK_PIN( ADC_INPUT_1 ) 			// declare pin named NRK_ADC_INPUT_1
DECLARE_NRK_PIN( ADC_INPUT_2 ) 			// declare pin named NRK_ADC_INPUT_2
DECLARE_NRK_PIN( ADC_INPUT_3 ) 
DECLARE_NRK_PIN( ADC_INPUT_4 ) 
DECLARE_NRK_PIN( ADC_INPUT_5 ) 	
DECLARE_NRK_PIN( JTAG_TCK )				// declare pin named NRK_JTAG_TCK
DECLARE_NRK_PIN( JTAG_TMS )				// declare pin named NRK_JTAG_TMS
DECLARE_NRK_PIN( JTAG_TDO )				// declare pin named NRK_JTAG_TDO
DECLARE_NRK_PIN( JTAG_TDI )				// declare pin named NRK_JTAG_TDI

DECLARE_NRK_PIN( ADC_INPUT_6 ) 			// declare pin named NRK_ADC_INPUT_6
DECLARE_NRK_PIN( ADC_INPUT_7 ) 			// declare pin named NRK_ADC_INPUT_7

#endif
