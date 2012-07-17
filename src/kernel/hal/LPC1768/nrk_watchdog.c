//
// include this ...#include "mbed.h"
#include <include.h>
#include <nrk_watchdog.h>
#include <nrk_error.h>
#include <nrk.h>
/* Dalton Banks */
// #include <avr/wdt.h>


/* --------------------- BIT DEFINITIONS -------------------------------------- */
/** WDT interrupt enable bit */
#define WDT_WDMOD_WDEN			    ((uint32_t)(1<<0))
/** WDT interrupt enable bit */
#define WDT_WDMOD_WDRESET			((uint32_t)(1<<1))
/** WDT time out flag bit */
#define WDT_WDMOD_WDTOF				((uint32_t)(1<<2))
/** WDT Time Out flag bit */
#define WDT_WDMOD_WDINT				((uint32_t)(1<<3))
/** WDT Mode */
#define WDT_WDMOD(n)				((uint32_t)(1<<1))

/** Define divider index for microsecond ( us ) */
#define WDT_US_INDEX	((uint32_t)(1000000))
/** WDT Time out minimum value */
#define WDT_TIMEOUT_MIN	((uint32_t)(0xFF))
/** WDT Time out maximum value */
#define WDT_TIMEOUT_MAX	((uint32_t)(0xFFFFFFFF))

/** Watchdog mode register mask */
#define WDT_WDMOD_MASK			(uint8_t)(0x02)
/** Watchdog timer constant register mask */
#define WDT_WDTC_MASK			(uint8_t)(0xFFFFFFFF)
/** Watchdog feed sequence register mask */
#define WDT_WDFEED_MASK 		(uint8_t)(0x000000FF)
/** Watchdog timer value register mask */
#define WDT_WDCLKSEL_MASK 		(uint8_t)(0x03)
/** Clock selected from internal RC */
#define WDT_WDCLKSEL_RC			(uint8_t)(0x00)
/** Clock selected from PCLK */
#define WDT_WDCLKSEL_PCLK		(uint8_t)(0x01)
/** Clock selected from external RTC */
#define WDT_WDCLKSEL_RTC		(uint8_t)(0x02)

/* ---------------- CHECK PARAMETER DEFINITIONS ---------------------------- */
/* Macro check clock source selection  */
#define PARAM_WDT_CLK_OPT(OPTION)  ((OPTION ==WDT_CLKSRC_IRC)||(OPTION ==WDT_CLKSRC_IRC)\
||(OPTION ==WDT_CLKSRC_IRC))

/* Macro check WDT mode */
#define PARAM_WDT_MODE_OPT(OPTION)  ((OPTION ==WDT_MODE_INT_ONLY)||(OPTION ==WDT_MODE_RESET))
/**
 * @}
 */


/* Public Types --------------------------------------------------------------- */
/** @defgroup WDT_Public_Types WDT Public Types
 * @{
 */

/** @brief Clock source option for WDT */
typedef enum {
	WDT_CLKSRC_IRC = 0, /*!< Clock source from Internal RC oscillator */
	WDT_CLKSRC_PCLK = 1, /*!< Selects the APB peripheral clock (PCLK) */
	WDT_CLKSRC_RTC = 2 /*!< Selects the RTC oscillator */
} WDT_CLK_OPT;

/** @brief WDT operation mode */
typedef enum {
	WDT_MODE_INT_ONLY = 0, /*!< Use WDT to generate interrupt only */
	WDT_MODE_RESET = 1    /*!< Use WDT to generate interrupt and reset MCU */
} WDT_MODE_OPT;

/*Private Functions -----------------------------------------------------------*/
static void WDT_Feed (void);

void WDT_Feed (void)
{
	// Disable irq interrupt
	__disable_irq(); //asm code in 
	LPC_WDT->WDFEED = 0xAA;
	LPC_WDT->WDFEED = 0x55;
	// Then enable irq interrupt
	__enable_irq();
}

/* End of Private Functions-----------------------------------------------------*/

/* Does not work for LPC1768...
 excerpt from User manual.."Once the WDEN and/or WDRESET bits are set they can not be cleared by software. Both flags are cleared by an external reset or a Watchdog timer underflow."
 */
void nrk_watchdog_disable()
{
    /*
    nrk_int_disable();
    nrk_watchdog_reset();
    MCUSR &= ~(1<<WDRF);
    WDTCSR |= (1<<WDCE) | (1<<WDE);
    WDTCSR = 0;
    nrk_int_enable();
     */
}

void nrk_watchdog_enable()
{
    LPC_WDT->WDCLKSEL = WDT_CLKSRC_PCLK;    // Set CLK src to PCLK
    uint32_t clk = SystemCoreClock / 16;    // WD has a fixed /4 prescaler, PCLK default is /4
    LPC_WDT->WDTC = 10 * (float)clk;        // 10 secs timeout...
    LPC_WDT->WDMOD = 0x3;  
    WDT_Feed();
}



int8_t nrk_watchdog_check()
{

    if(((LPC_SC->RSID >> 2) & 1)==0) return NRK_OK;
    return NRK_ERROR;
}

inline void nrk_watchdog_reset()
{
    WDT_Feed();
}
