#include <include.h>
#include <nrk_status.h>
#include <nrk_error.h>
/*
 include these
 #include "mbed.h" 
 #include "LPC17xx.h"
*/

uint8_t _nrk_startup_error()
{
uint8_t error;
error=0;
// Use the timer settings that are normally 0 on reset to detect
// if the OS has reboot by accident


// Check Watchdog timer
if((LPC_SC->RSID >> 2) & 1)
	{
	// don't clear wdt
        LPC_SC->RSID |= (1<<2);
        error|=0x10;
	}


// Check Brown Out 
if(((LPC_SC->RSID >> 3) & 1) && (!(LPC_SC->RSID & 1)))
	{
        LPC_SC->RSID |= 0x04;     
        error|=0x04;
	}

    
// Check External Reset 
if (((LPC_SC->RSID >> 1) & 1))
    {
        LPC_SC->RSID |= (1<<1);
        error|=0x02;
    }
    


// If any of the above errors went off, then the next errors will
// incorrectly be set!  So make sure to bail early!
if(error!=0) 
    return error;
else
    return 0x01;
    /* not confirmed yet...
// Check if normal power up state is set and then clear it
if( (MCUSR & (1<<PORF)) != 0 )
	{
	MCUSR &= ~(1<<PORF);
	}
	else {
	error|=0x01;
	}
     */
// check uart state 
//if((volatile uint8_t)TCCR2A!=0) error|=0x01;

    //return error;
}


