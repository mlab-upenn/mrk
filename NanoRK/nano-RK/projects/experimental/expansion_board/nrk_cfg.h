/***************************************************************
*                            NanoRK CONFIG                     *
***************************************************************/
#ifndef __nrk_cfg_h	
#define __nrk_cfg_h


// NRK_REPORT_ERRORS will cause the kernel to print out information about
// missed deadlines or reserve violations
#define NRK_REPORT_ERRORS
// NRK_HALT_ON_ERRORS will cause the kernel to freeze on errors so that
// it is easier to see debugging messages.
//#define NRK_HALT_AND_LOOP_ON_ERROR


#define NRK_WATCHDOG
#define NRK_REBOOT_ON_ERROR


// If you want to disable the looping kernel error message when the system
// halts, then include the following define.  This will automatically
// halt the error as well.
//#define NRK_HALT_ON_ERROR
#define IGNORE_BROWN_OUT_ERROR
#define IGNORE_EXT_RST_ERROR


#define IGNORE_EXT_RST_ERROR
#define IGNORE_BROWN_OUT_ERROR

#define NRK_NO_BOUNDED_CONTEXT_SWAP


// NRK_STACK_CHECK adds a little check to see if the bottom of the stack
// has been over written on all suspend calls
#define NRK_STACK_CHECK

//#define KERNEL_STK_ARRAY
// Leave NRK_NO_POWER_DOWN define in if the target can not wake up from sleep 
// because it has no asynchronously clocked
#define NRK_NO_POWER_DOWN
#define NRK_UART_BUF 1
#define MAX_RX_UART_BUF 96
#define NRK_UART1_BUF 1

#define RADIO_PRIORITY_CEILING          10



// Max number of tasks in your application
// Be sure to include the idle task
// Making this the correct size will save on BSS memory which
// is both RAM and ROM...
#define NRK_MAX_TASKS       		6

// We have at most NRK_DRIVER_CNT drivers to register
#define NRK_MAX_DRIVER_CNT		1    

#define	NRK_N_RES			0	
                           

#define NRK_TASK_IDLE_STK_SIZE         128 // Idle task stack size min=32 
#define NRK_APP_STACKSIZE              256
#define BMAC_STACKSIZE                 256
#define NRK_KERNEL_STACKSIZE           196 

#define NRK_MAX_RESOURCE_CNT           4



#endif
