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
#include <TWI_Master.h>

#define PKT_SIZE	10

#define TWI_GEN_CALL         0x00       // The General Call address is 0

// Sample TWI transmission commands
#define TWI_CMD_MASTER_WRITE 0x10
#define TWI_CMD_MASTER_READ  0x20

// Sample TWI transmission states, used in the main application.
#define SEND_DATA             0x01
#define REQUEST_DATA          0x02
#define READ_DATA_FROM_BUFFER 0x03

unsigned char TWI_Act_On_Failure_In_Last_Transmission (unsigned char
                                                       TWIerrorMsg)
{
  // A failure has occurred, use TWIerrorMsg to determine the nature of the failure
  // and take appropriate actions.
  // Se header file for a list of possible failures messages.

  // Here is a simple sample, where if received a NACK on the slave address,
  // then a retransmission will be initiated.

  if ((TWIerrorMsg == TWI_MTX_ADR_NACK) | (TWIerrorMsg == TWI_MRX_ADR_NACK))
    TWI_Start_Transceiver ();

  return TWIerrorMsg;
}




NRK_STK Stack1[NRK_APP_STACKSIZE];
nrk_task_type TaskOne;
void Task1 (void);

NRK_STK Stack2[NRK_APP_STACKSIZE];
nrk_task_type TaskTwo;
void Task2 (void);


void nrk_create_taskset ();

int main ()
{
  nrk_setup_ports ();
  nrk_setup_uart (UART_BAUDRATE_115K2);

  nrk_init ();

  nrk_led_clr (ORANGE_LED);
  nrk_led_clr (BLUE_LED);
  nrk_led_clr (GREEN_LED);
  nrk_led_clr (RED_LED);

  nrk_time_set (0, 0);
  nrk_create_taskset ();
  nrk_start ();

  return 0;
}

uint8_t messageBuf[32];

void Task1 ()
{
  nrk_time_t t;
  uint8_t seq_num, last_seq_num;
  uint8_t val, i, checksum;
  unsigned char TWI_targetSlaveAddress, temp, TWI_operation = 0,
    pressedButton, myCounter = 0;


  nrk_kprintf (PSTR ("Nano-RK Version "));
  printf ("%d\r\n", NRK_VERSION);

  nrk_kprintf (PSTR ("I2C Master\r\n"));
  nrk_kprintf (PSTR ("Green LED indicated transfer\r\n"));
  nrk_kprintf (PSTR ("Reset FF Meter (i2c slave) first and then master to start system\r\n"));
  nrk_kprintf (PSTR ("Dropped messages and failed checksums are reported\r\n"));
  TWI_Master_Initialise ();
  //__enable_interrupt();
  sei ();

  TWI_targetSlaveAddress = 0x10;
  // This example code runs forever; sends a byte to the slave, then requests a byte
  // from the slave and stores it on PORTB, and starts over again. Since it is interupt
  // driven one can do other operations while waiting for the transceiver to complete.

  // Send initial data to slave
  messageBuf[0] =
    (TWI_targetSlaveAddress << TWI_ADR_BITS) | (FALSE << TWI_READ_BIT);
  messageBuf[1] = 0x01;
  TWI_Start_Transceiver_With_Data (messageBuf, 2);

  TWI_operation = REQUEST_DATA; // Set the next operation

  for (;;) {
    // Check if the TWI Transceiver has completed an operation.
    if (!TWI_Transceiver_Busy ()) {
      // Check if the last operation was successful
      if (TWI_statusReg.lastTransOK) {
        nrk_led_clr (RED_LED);
        // nrk_kprintf( PSTR("tx ok\r\n") );
        // Determine what action to take now
        if (TWI_operation == SEND_DATA) {       // Send data to slave
          messageBuf[0] =
            (TWI_targetSlaveAddress << TWI_ADR_BITS) | (FALSE <<
                                                        TWI_READ_BIT);
          messageBuf[1] = 0x55;
          TWI_Start_Transceiver_With_Data (messageBuf, 2);

          TWI_operation = REQUEST_DATA; // Set next operation
        }
        else if (TWI_operation == REQUEST_DATA) {       // Request data from slave
          messageBuf[0] =
            (TWI_targetSlaveAddress << TWI_ADR_BITS) | (TRUE << TWI_READ_BIT);
          TWI_Start_Transceiver_With_Data (messageBuf, PKT_SIZE+2);

          TWI_operation = READ_DATA_FROM_BUFFER;        // Set next operation        
        }
        else if (TWI_operation == READ_DATA_FROM_BUFFER) {      // Get the received data from the transceiver buffer
          TWI_Get_Data_From_Transceiver (messageBuf, PKT_SIZE+2);
          //seq_num = messageBuf[1] << 8 | messageBuf[2];
          seq_num = messageBuf[1];
          if (seq_num != (uint8_t)(last_seq_num + 1))
	  {
 	    if(seq_num==0 && last_seq_num==0) nrk_kprintf(PSTR("Packets started!\r\n"));
	    else printf ("Lost packet: %u vs %u\r\n", last_seq_num, seq_num);
	  }
          checksum = 0;
          for (i = 1; i < PKT_SIZE+1; i++)
            checksum += messageBuf[i];

          if (checksum != messageBuf[PKT_SIZE+1]) {
            printf ("Checksum failed: %u vs %u\r\n", checksum,
                    messageBuf[PKT_SIZE+1]);

            for (i = 0; i < PKT_SIZE+2; i++)
              printf ("%d ", messageBuf[i]);
            printf ("\r\n");
          }
          last_seq_num = seq_num;
          nrk_led_toggle (GREEN_LED);
          //                              nrk_kprintf(PSTR( "rx: " ));
          //for(i=1; i<3; i++ ) printf( "%d ",messageBuf[i]);
          //nrk_kprintf(PSTR("\r\n" ));
          //for(i=0; i<9; i++ )
          // printf( "%d ",messageBuf[i]);
          //                              nrk_kprintf(PSTR( "\r\n" ));
          //messageBuf[1]++;
          //nrk_wait_until_next_period();
          //TWI_operation = SEND_DATA;    // Set next operation        
          TWI_operation = REQUEST_DATA; // Set next operation        
        }
      }
      else                      // Got an error during the last transmission
      {
        nrk_led_set (RED_LED);
        //nrk_kprintf( PSTR("tx err\r\n") );
        // Use TWI status information to detemine cause of failure and take appropriate actions. 
        TWI_Act_On_Failure_In_Last_Transmission (TWI_Get_State_Info ());
      }
    }

    //else
    //nrk_kprintf( PSTR("tx busy\r\n") );
    // Do something else while waiting for the TWI Transceiver to complete the current operation
    //__no_operation(); // Put own code here.
  }

}

void Task2 ()
{
  int16_t cnt;
  printf ("Task2 PID=%u\r\n", nrk_get_pid ());
  cnt = 0;
  while (1) {
    nrk_led_toggle (BLUE_LED);
    nrk_wait_until_next_period ();
  }
}


void nrk_create_taskset ()
{
  nrk_task_set_entry_function (&TaskOne, Task1);
  nrk_task_set_stk (&TaskOne, Stack1, NRK_APP_STACKSIZE);
  TaskOne.prio = 1;
  TaskOne.FirstActivation = TRUE;
  TaskOne.Type = BASIC_TASK;
  TaskOne.SchType = PREEMPTIVE;
  TaskOne.period.secs = 0;
  TaskOne.period.nano_secs = 250 * NANOS_PER_MS;
  TaskOne.cpu_reserve.secs = 0;
  TaskOne.cpu_reserve.nano_secs = 0;
  TaskOne.offset.secs = 0;
  TaskOne.offset.nano_secs = 0;
  nrk_activate_task (&TaskOne);

  /*
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
   */

}
