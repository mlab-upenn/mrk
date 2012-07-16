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
#include <avr/sleep.h>
#include <hal.h>
#include <tdma_asap.h>
#include <tdma_asap_scheduler.h>
#include <nrk_error.h>


NRK_STK my_task_stack[NRK_APP_STACKSIZE];
nrk_task_type myTask;

void nrk_create_taskset ();

uint8_t rx_buf[RF_MAX_PAYLOAD_SIZE];
uint8_t tx_buf[RF_MAX_PAYLOAD_SIZE];

int main ()
{
  nrk_setup_ports ();
  nrk_setup_uart (UART_BAUDRATE_115K2);

  nrk_init ();

  nrk_led_clr (0);
  nrk_led_clr (1);
  nrk_led_clr (2);
  nrk_led_clr (3);

  nrk_time_set (0, 0);

  tdma_mode_set(TDMA_MASTER);

  // do the scheduling by yourself
  tdma_schedule_method_set(TDMA_SCHED_MANUAL);

  // send sync to child
  tdma_schedule_add(0, TDMA_TX_CHILD, 0);

  // SCHEDULE A
  // add a RX from child every tenth slot
  for (tdma_slot_t i = 10; i < TDMA_SLOTS_PER_CYCLE; i+=10)
  {
    tdma_schedule_add(i, TDMA_RX, 0);
  }
  
  
  // SCHEDULE B
  // rcv data from child. priority is ignored
  //tdma_schedule_add(10, TDMA_RX, -1);

  // SCHEDULE C: Sync Test
  // Add 2 RX slots from children from level 1 and 2
  //tdma_schedule_add(0, TDMA_TX_CHILD, 0);
  //tdma_schedule_add(10, TDMA_RX, 0);
  //tdma_schedule_add(20, TDMA_RX, 0);


  tdma_task_config ();

  nrk_create_taskset ();

  nrk_start ();

  return 0;
}

void Task1()
{
    uint8_t * local_rx_buf;
    uint8_t my_addr8;
    uint8_t my_level;
    int8_t rssi;
    uint8_t length;
    uint16_t slot;
    uint8_t cnt = 0;


    tdma_init(10);

    while(!tdma_started())
        nrk_wait_until_next_period();

    //tdma_schedule_print();

    //my_addr8 = tdma_mac_get();
    my_addr8 = 1;
    //my_level = tdma_tree_level_get();
    my_level = 0;

    nrk_kprintf(PSTR("Starting task!\r\n"));

    while(1)
    {
        if (tdma_rx_pkt_check() != 0)
        {
            // I have a packet
            local_rx_buf = tdma_rx_pkt_get(&length, &rssi, &slot);
            printf("Got pkt len %d rssi %d slot %d\r\n", length, rssi, slot);

            for (uint8_t i = TDMA_DATA_START; i < length; i++)
            {
                printf("%c", local_rx_buf[i]);
            }
            nrk_kprintf(PSTR("\r\n"));
            tdma_rx_pkt_release();

        }

/*
        if (tdma_tx_pkt_check() == 0)
        {
            sprintf(&tx_buf[TDMA_DATA_START], "From %d lvl %d cnt %d\r\n",
                 my_addr8, my_level, cnt);

            length = strlen(&tx_buf[TDMA_DATA_START] + TDMA_DATA_START);
            tdma_tx_pkt(tx_buf, length);

            cnt++;

        }

        tdma_wait_until_rx_or_tx();
*/
    }
}

void
nrk_create_taskset()
{
  
  myTask.task = Task1;
  //nrk_task_set_stk( &myTask, tx_task_stack, NRK_APP_STACKSIZE);
  myTask.Ptos = (void *) &my_task_stack[NRK_APP_STACKSIZE-1];
  myTask.Pbos = (void *) &my_task_stack[0];
  myTask.prio = 2;
  myTask.FirstActivation = TRUE;
  myTask.Type = BASIC_TASK;
  myTask.SchType = PREEMPTIVE;
  myTask.period.secs = 1;
  myTask.period.nano_secs = 0;
  myTask.cpu_reserve.secs = 0;
  myTask.cpu_reserve.nano_secs = 0;
  myTask.offset.secs = 2;
  myTask.offset.nano_secs = 0;
  nrk_activate_task (&myTask);


  //nrk_kprintf ( PSTR("Create done\r\n") );
}

