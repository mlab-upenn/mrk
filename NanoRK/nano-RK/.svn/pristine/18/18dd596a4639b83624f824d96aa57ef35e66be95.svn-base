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
*  Authors: 
*  Patrick Lazik
*  Niranjini Rajagopal
*
*******************************************************************************/


#include <nrk.h>
#include <include.h>
#include <ulib.h>
#include <stdio.h>
#include <avr/sleep.h>
#include <hal.h>
#include <nrk_error.h>
#include <nrk_timer.h>
#include <nrk_stack_check.h>
#include <basic_rf.h>

#include <nrk_driver_list.h>
#include <nrk_driver.h>
#include <slip.h>
#include <ade7878.h>
#include <TWI_slave.h>


// Sample TWI transmission commands
#define TWI_CMD_MASTER_WRITE 0x10
#define TWI_CMD_MASTER_READ  0x20

unsigned char TWI_slaveAddress;

// When there has been an error, this function is run and takes care of it
unsigned char TWI_Act_On_Failure_In_Last_Transmission (unsigned char
                                                       TWIerrorMsg)
{
  // A failure has occurred, use TWIerrorMsg to determine the nature of the failure
  // and take appropriate actions.
  // Se header file for a list of possible failures messages.

  // This very simple example puts the error code on PORTB and restarts the transceiver with
  // all the same data in the transmission buffers.
  PORTB = TWIerrorMsg;
  TWI_Start_Transceiver ();

  return TWIerrorMsg;
}

uint8_t i2c_buf[32];




NRK_STK Stack1[NRK_APP_STACKSIZE];
nrk_task_type TaskOne;
void Task1 (void);              // To initialize timer

NRK_STK Stack2[NRK_APP_STACKSIZE];
nrk_task_type TaskTwo;
void Task2 (void);              // Slow display of RMS values

// Magic memory alignment?
nrk_task_type TaskFour;

void nrk_create_taskset ();
//ADE7878 related............................

uint32_t V_BasicRdChk_U32R;
uint32_t V_ArmsCurr_U32R, V_ArmsVolt_U32R;
int32_t V_ArmsWatt_S32R;

int32_t V_Awatt_S32R = 0, V_Avar_S32R = 0;
uint8_t F_ReadyForSignal_U8R = 1, F_1secData_U8R = 0;
uint16_t V_RdDSP_U16R;
uint8_t F_RdDSP_U8R = 0;
uint32_t V_DSPrstCnt_U32R = 0;
int32_t V_RdParam1_S32R = 0, V_RdParam2_S32R = 0, V_RdParam3_S32R = 0;
uint32_t V_CalcTemp1_U32R, V_CalcTemp2_U32R;
int32_t V_CalcTemp1_S32R, V_CalcTemp2_S32R;

////////////////////////////////
uint32_t V_DataOld_U32R = 0, V_DataNew_U32R = 0, V_EvnDetCntr_U32R = 0;
int32_t V_DiffData_S32R;
uint8_t F_Det_U8R = 0, F_Occ_U8R = 0;

uint8_t V_DummyCnt_U32R = 0, A_1secRmsVals_U8R[20], F_1secDataReady_U8R =
  0, V_1secDummyCnt_U32R = 0;
uint32_t V_Dummy32a_U32R = 0, V_Dummy32b_U32R = 0;
uint32_t V_TINTcnt_U32R = 0;
///////////////////////////////
//End of ADE7878.............................


#define CIRC_BUF_ELEMENTS       1024	
#define CIRC_BUF_ELEMENT_SIZE 	10
#define CIRC_BUF_SIZE		(CIRC_BUF_ELEMENTS*CIRC_BUF_ELEMENT_SIZE)

uint8_t circ_buf[CIRC_BUF_SIZE];
uint16_t read_index, write_index;
uint8_t seq_num;

int main ()
{
  uint8_t t;
  uint8_t cnt, i, j, length;
  uint32_t dcnt = 0;

  //Initializig ports
  nrk_setup_ports ();
  nrk_setup_uart (UART_BAUDRATE_115K2);
  nrk_kprintf (PSTR ("Starting up...\r\n"));
  nrk_led_clr (RED_LED);
  nrk_led_clr (BLUE_LED);
  nrk_led_clr (ORANGE_LED);
  nrk_led_clr (GREEN_LED);

  ade_nrk_setup ();

  //Initialize NRK        
  nrk_init ();
  nrk_time_set (0, 0);
  nrk_create_taskset ();
  nrk_start ();
  return 0;
}

//This timer interrupt occurs every 1ms. We read the Watt, VAR registers and write to buffer
void my_timer_callback ()
{
  int8_t i, v;

  //V_RdParam1_S32R       =       ade_read32(AWATT);
  //V_RdParam2_S32R               =       ade_read32(AVAR);
  //V_Awatt_S32R = ade_read32(AWATT);
  V_RdParam1_S32R = ade_read32 (IAWV);
  V_RdParam2_S32R = ade_read32 (VAWV);
  V_RdParam3_S32R = ade_read32 (IBWV);

  //nrk_led_set(RED_LED);

  //Write the 6 new bytes to the buffer and increment Wr pointer
//  circ_buf[write_index] = (seq_num >> 8) & 0xff;
//  circ_buf[write_index + 1] = seq_num & 0xff;
  circ_buf[write_index] = seq_num & 0xff;
  memcpy (&circ_buf[write_index + 1], &V_RdParam1_S32R, 3);
  memcpy (&circ_buf[write_index + 1 + 3], &V_RdParam2_S32R, 3);
  memcpy (&circ_buf[write_index + 1 + 6], &V_RdParam3_S32R, 3);
  write_index += CIRC_BUF_ELEMENT_SIZE;

  if (write_index >= CIRC_BUF_SIZE)
    write_index = 0;
  //Every 1 second, we read RMS values. This is done inside the ISR and not as a separate task because both - the ISR and the lines below access the 7878 through SPI and we cant let one SPI instr interrupt another SPI instr
  if (seq_num % 1000 == 0) {
    V_RdDSP_U16R = ade_read16 (RUN);
    if (V_RdDSP_U16R == 0) {
      V_DSPrstCnt_U32R++;
      ade_write16 (RUN, START); //if the DSP got reset, start it so that it continues to run
    }
  }
  seq_num++;
}

//This task initializes the Timer
void Task1 ()
{
  uint16_t cnt;
  uint8_t val, i, checksum;

  printf ("My node's address is %d\r\n", NODE_ADDR);

  printf ("Task1 PID=%d\r\n", nrk_get_pid ());
  cnt = 0;

  // Setup circular buffer for i2c data
  read_index = 0;
  write_index = 0;
  seq_num = 0;
  // Setup application timer with:
  //       Prescaler = 2 
  //       Compare Match = 2000
  //       Sys Clock = 16000 MHz
  // Prescaler 2 means divide sys clock by 8
  // 16000000 / 8 = 2MHz clock
  // 1 / 2MHz = 0.5 us per tick
  // 0.5 us * 2000 = 1 ms / per interrupt callback

  val = nrk_timer_int_configure (NRK_APP_TIMER_0, 2, 1000, &my_timer_callback); //value 2000 is for 1ms. Change to 16000 for 8ms
  if (val == NRK_OK)
    nrk_kprintf (PSTR ("Callback timer setup\r\n"));
  else
    nrk_kprintf (PSTR ("Error setting up timer callback\r\n"));

  // Zero the timer
  nrk_timer_int_reset (NRK_APP_TIMER_0);
  // Start the timer
  nrk_timer_int_start (NRK_APP_TIMER_0);



  while (1) {
    // Own TWI slave address
    TWI_slaveAddress = 0x10;

    // Initialise TWI module for slave operation. Include address and/or enable General Call.
    TWI_Slave_Initialise ((unsigned char) ((TWI_slaveAddress << TWI_ADR_BITS)
                                           | (TRUE << TWI_GEN_BIT)));

    nrk_int_enable ();

    nrk_kprintf (PSTR ("slave start\r\n"));

    TWI_Start_Transceiver ();

    for (;;) {
      if (!TWI_Transceiver_Busy ()) {
        if (TWI_statusReg.RxDataInBuf) {
          TWI_Get_Data_From_Transceiver (&i2c_buf, 2);
          nrk_kprintf (PSTR ("slave got: "));
          for (i = 0; i < 2; i++)
            printf ("%d ", i2c_buf[i]);
          nrk_kprintf (PSTR ("\r\n"));
        }

        while (read_index == write_index)
          nrk_wait_until_next_period ();

        checksum = 0;
        for (i = 0; i < CIRC_BUF_ELEMENT_SIZE; i++) {
          i2c_buf[i] = circ_buf[read_index + i];
	   checksum += i2c_buf[i];
        }
        // checksum is last packet
        i2c_buf[CIRC_BUF_ELEMENT_SIZE] = checksum;
        TWI_Start_Transceiver_With_Data (&i2c_buf, CIRC_BUF_ELEMENT_SIZE + 1);


        read_index += CIRC_BUF_ELEMENT_SIZE;
        if (read_index >= CIRC_BUF_SIZE)
          read_index = 0;


      }
    }





  }
}

// Slow display of RMS values
// This task runs every 100ms and checks if RMS data is available to be printed.
// Actually, it should be enough to call this every 1 second. But the data goes through the RF tx, to the other node RF Rx, and then through RS232 to the SLIP Server on PC, and I observed that the SLIP Server sometimes missed the display of a few seconds data, so doing this every 100ms.
void Task2 ()
{
  uint8_t cnt;
  printf ("Task2 PID=%d\r\n", nrk_get_pid ());
  cnt = 0;

  while (1) {
    nrk_led_toggle (BLUE_LED);
    nrk_wait_until_next_period ();
    //cnt++;
  }
}






void nrk_create_taskset ()
{
  TaskOne.task = Task1;         // To initialize timer
  TaskOne.Ptos = (void *) &Stack1[NRK_APP_STACKSIZE];
  TaskOne.Pbos = (void *) &Stack1[0];
  TaskOne.prio = 1;
  TaskOne.FirstActivation = TRUE;
  TaskOne.Type = BASIC_TASK;
  TaskOne.SchType = PREEMPTIVE;
  TaskOne.period.secs = 0;
  TaskOne.period.nano_secs = 2 * NANOS_PER_MS;
  TaskOne.cpu_reserve.secs = 0;
  TaskOne.cpu_reserve.nano_secs = 0;
  TaskOne.offset.secs = 0;
  TaskOne.offset.nano_secs = 0;
  nrk_activate_task (&TaskOne);

  TaskTwo.task = Task2;         // Slow display of RMS values
  TaskTwo.Ptos = (void *) &Stack2[NRK_APP_STACKSIZE];
  TaskTwo.Pbos = (void *) &Stack2[0];
  TaskTwo.prio = 3;
  TaskTwo.FirstActivation = TRUE;
  TaskTwo.Type = BASIC_TASK;
  TaskTwo.SchType = PREEMPTIVE;
  TaskTwo.period.secs = 1;
  TaskTwo.period.nano_secs = 0 * NANOS_PER_MS;
  TaskTwo.cpu_reserve.secs = 0;
  TaskTwo.cpu_reserve.nano_secs = 30 * NANOS_PER_MS;
  TaskTwo.offset.secs = 0;
  TaskTwo.offset.nano_secs = 0;
  nrk_activate_task (&TaskTwo);




}
