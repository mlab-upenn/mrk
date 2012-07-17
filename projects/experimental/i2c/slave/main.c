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
*******************************************************************************/

#include <nrk.h>
#include <include.h>
#include <ulib.h>
#include <stdio.h>
#include <hal.h>
#include <nrk_error.h>
#include <nrk_timer.h>
#include <nrk_stack_check.h>
#include <nrk_stats.h>
#include <TWI_slave.h>


// Sample TWI transmission commands
#define TWI_CMD_MASTER_WRITE 0x10
#define TWI_CMD_MASTER_READ  0x20

// When there has been an error, this function is run and takes care of it
unsigned char TWI_Act_On_Failure_In_Last_Transmission ( unsigned char TWIerrorMsg );



NRK_STK Stack1[NRK_APP_STACKSIZE];
nrk_task_type TaskOne;
void Task1(void);

NRK_STK Stack2[NRK_APP_STACKSIZE];
nrk_task_type TaskTwo;
void Task2 (void);


void nrk_create_taskset();



int
main ()
{
  nrk_setup_ports();
  nrk_setup_uart(UART_BAUDRATE_115K2);
  nrk_init();

  nrk_led_clr(ORANGE_LED);
  nrk_led_clr(BLUE_LED);
  nrk_led_clr(GREEN_LED);
  nrk_led_clr(RED_LED);
 
  nrk_time_set(0,0);
  nrk_create_taskset ();
  nrk_start();
  
  return 0;
}



unsigned char TWI_Act_On_Failure_In_Last_Transmission ( unsigned char TWIerrorMsg )
{
                    // A failure has occurred, use TWIerrorMsg to determine the nature of the failure
                    // and take appropriate actions.
                    // Se header file for a list of possible failures messages.
  
                    // This very simple example puts the error code on PORTB and restarts the transceiver with
                    // all the same data in the transmission buffers.
  //PORTB = TWIerrorMsg;
  TWI_Start_Transceiver();
                    
  return TWIerrorMsg; 
}

uint8_t buf[16];

void Task1()
{
nrk_time_t t;
uint16_t cnt;
uint8_t val,i;
unsigned char messageBuf[TWI_BUFFER_SIZE];
unsigned char TWI_slaveAddress;
uint8_t temp;

cnt=0;


nrk_kprintf( PSTR("Nano-RK Version ") );
printf( "%d\r\n",NRK_VERSION );
  
  // Own TWI slave address
  TWI_slaveAddress = 0x10;

  // Initialise TWI module for slave operation. Include address and/or enable General Call.
  TWI_Slave_Initialise( (unsigned char)((TWI_slaveAddress<<TWI_ADR_BITS) | (TRUE<<TWI_GEN_BIT) )); 
                       
  nrk_int_enable();

  nrk_kprintf( PSTR("slave start\r\n") );

  TWI_Start_Transceiver( );    
         
	    for(i=0; i< 10; i++ ) buf[i]=i;
  for(;;)
  {
    if ( ! TWI_Transceiver_Busy() )                              
    {
      if ( TWI_statusReg.RxDataInBuf )
      {
        TWI_Get_Data_From_Transceiver(&buf, 2);  
        //TWI_Get_Data_From_Transceiver(&buf, 10);  
	nrk_kprintf( PSTR("slave got: " ));
	for(i=0; i<2; i++ ) printf( "%d ",buf[i] );
	nrk_kprintf( PSTR("\r\n" ));
      }
			temp++;
	    for(i=0; i< 10; i++ ) buf[i]++;
      TWI_Start_Transceiver_With_Data(&buf, 10); 
    } 
    // nrk_kprintf( PSTR("slave busy\r\n") );
	  // nrk_wait_until_next_period();
  }

}

void Task2()
{
  int16_t cnt;
  printf( "Task2 PID=%u\r\n",nrk_get_pid());
  cnt=0;
  while(1) {
	nrk_led_toggle(RED_LED);
	nrk_wait_until_next_period();
	}
}


void
nrk_create_taskset()
{
  nrk_task_set_entry_function( &TaskOne, Task1);
  nrk_task_set_stk( &TaskOne, Stack1, NRK_APP_STACKSIZE);
  TaskOne.prio = 1;
  TaskOne.FirstActivation = TRUE;
  TaskOne.Type = BASIC_TASK;
  TaskOne.SchType = PREEMPTIVE;
  TaskOne.period.secs = 0;
  TaskOne.period.nano_secs = 250*NANOS_PER_MS;
  TaskOne.cpu_reserve.secs = 0;
  TaskOne.cpu_reserve.nano_secs = 0;
  TaskOne.offset.secs = 0;
  TaskOne.offset.nano_secs= 0;
  nrk_activate_task (&TaskOne);

  nrk_task_set_entry_function( &TaskTwo, Task2);
  nrk_task_set_stk( &TaskTwo, Stack2, NRK_APP_STACKSIZE);
  TaskTwo.prio = 2;
  TaskTwo.FirstActivation = TRUE;
  TaskTwo.Type = BASIC_TASK;
  TaskTwo.SchType = PREEMPTIVE;
  TaskTwo.period.secs = 0;
  TaskTwo.period.nano_secs = 500*NANOS_PER_MS;
  TaskTwo.cpu_reserve.secs = 0;
  TaskTwo.cpu_reserve.nano_secs = 100*NANOS_PER_MS;
  TaskTwo.offset.secs = 0;
  TaskTwo.offset.nano_secs= 0;
  nrk_activate_task (&TaskTwo);


}



