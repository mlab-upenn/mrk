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
*  Zane Starr
*  Anand Eswaren
*******************************************************************************/

#include <include.h>
#include <nrk.h>
/* Dalton Banks */
// #include <avr/sleep.h>
#include <nrk_stack_check.h>
#include <nrk_task.h>
#include <nrk_cfg.h>
#include <nrk_timer.h>
#include <nrk_error.h>

Serial pc2(USBTX, USBRX); // tx, rx

#define BUILD_DATE "Date: " __DATE__ "\n"
/* Constants required to set up the initial stack. */
#define INITIAL_XPSR			( 0x01000000 )

/*
*********************************************************************************************************
*                                        INITIALIZE A TASK'S STACK
*
* Description: This function is highly processor specific.
*
* Arguments  : task          is a pointer to the task code
*
*              pdata         is a pointer to a user supplied data area that will be passed to the task
*                            when the task first executes.
*
*              ptos          is a pointer to the top of stack.  It is assumed that 'ptos' points to
*                            a 'free' entry on the task stack.  
*                            'ptos' contains the HIGHEST valid address of the stack.  
*
*              opt           specifies options that can be used to alter the behavior of OSTaskStkInit().
*                            We don't use have any option implemented for this project. You can just
*                            set opt to 0
*
* Returns    : Always returns the location of the new top-of-stack' once the processor registers have
*              been placed on the stack in the proper order.
*
* Note(s)    : 
*********************************************************************************************************
*/


void nrk_battery_save()
{
    /*
#ifdef NRK_BATTERY_SAVE
 	_nrk_stop_os_timer();
        _nrk_set_next_wakeup(250);
        nrk_led_clr(0);
        nrk_led_set(1);
        nrk_led_clr(2);
        nrk_led_clr(3);
        SET_VREG_INACTIVE();
        nrk_sleep();
#endif
     */
}

void nrk_sleep()
{
    /*
    set_sleep_mode (SLEEP_MODE_PWR_SAVE);
    sleep_mode ();
     */
}

void nrk_idle()
{
    /*
    set_sleep_mode( SLEEP_MODE_IDLE);
    sleep_mode ();
     */
}

void nrk_task_set_entry_function( nrk_task_type *task, void (*func) (void) )
{
    task->task=func; //function pointer...
}

void nrk_task_set_stk( nrk_task_type *task, NRK_STK stk_base[], uint16_t stk_size )
{
    if(stk_size<32) nrk_error_add(NRK_STACK_TOO_SMALL);
    /* Dalton Banks - changed cast from void* to NRK_STK* */
    task->Ptos = (NRK_STK *) &stk_base[stk_size-1];
    /* Dalton Banks - changed cast from void* to NRK_STK* */
    task->Pbos = (NRK_STK *) &stk_base[0];

}

/* Dalton Banks - changed all stack pointers to NRK_STK types */
void *nrk_task_stk_init (void (*task)(), NRK_STK *ptos, NRK_STK *pbos)
{    
    NRK_STK *stk ;  // 4 bytes
    NRK_STK *stkc;  // 2 byte
    
    /* Dalton Banks - changed cast from unsigned char* to NRK_STK* */
    stk    = pbos;          /* Load stack pointer */ 
    /* Dalton Banks - changed cast from unsigned char* to NRK_STK* */
    stkc = stk;
    *stkc = STK_CANARY_VAL;                 // Flag for Stack Overflow    
    stk    = ptos;          /* Load stack pointer */

    /* Simulate the stack */
	stk--; /* Offset added to account for the way the MCU uses the stack on entry/exit of interrupts. */
	*stk = INITIAL_XPSR;	/* xPSR */
	stk--;
	*stk = ( unsigned int ) task;	/* PC */
	stk--;
	*stk = 0;	/* LR */
	stk -= 5;	/* R12, R3, R2 ,R1 and R0. */
	stk -= 8;	/* R11, R10, R9, R8, R7, R6, R5 and R4. */

    return stk;
}

/* Dalton Banks - changed all stack pointers to NRK_STK types */
void nrk_stack_pointer_init()
{
    pc2.printf("enter nrk_stack_pointer_init()\r\n");
    NRK_STK *stkc;
    #ifdef KERNEL_STK_ARRAY
            stkc = &nrk_kernel_stk[NRK_KERNEL_STACKSIZE-1];
            nrk_kernel_stk[0]=STK_CANARY_VAL;
            nrk_kernel_stk_ptr = &nrk_kernel_stk[NRK_KERNEL_STACKSIZE-1];
        #else
            stkc = (NRK_STK *)(NRK_KERNEL_STK_TOP-NRK_KERNEL_STACKSIZE);
            *stkc = STK_CANARY_VAL;
    pc2.printf("nrk_stack_pointer_init debug point\r\n");
            stkc = (NRK_STK *)NRK_KERNEL_STK_TOP;
            nrk_kernel_stk_ptr = (NRK_STK *)NRK_KERNEL_STK_TOP;
        #endif
        //figure out why this is done... Abhijeet
        //*stkc++ = (uint16_t)((uint16_t)_nrk_timer_tick>>8);
        //*stkc = (uint16_t)((uint16_t)_nrk_timer_tick&0xFF);
        *stkc = (NRK_STK)_nrk_timer_tick;
}


void nrk_stack_pointer_restore()
{
    unsigned char *stkc;

    #ifdef KERNEL_STK_ARRAY
            /* Dalton Banks - changed (uint16_t*) cast to (unsigned char*) */
            stkc = (unsigned char*)&nrk_kernel_stk[NRK_KERNEL_STACKSIZE-1];
    #else
            stkc = (unsigned char *)NRK_KERNEL_STK_TOP;
    #endif
            //figure out why this is done... Abhijeet
            //*stkc++ = (uint16_t)((uint16_t)_nrk_timer_tick>>8);
            //*stkc = (uint16_t)((uint16_t)_nrk_timer_tick&0xFF);
            *stkc = (uint32_t)_nrk_timer_tick;
}

/* start the target running */
void nrk_target_start(void)
{

      _nrk_setup_timer();
      //nrk_int_enable();  
      __enable_irq(); //enable interrupts
}

