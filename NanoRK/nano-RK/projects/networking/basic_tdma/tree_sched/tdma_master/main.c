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

#include <tdma_asap_tree.h>

// include tree creation functions
#define TDMA_TREEMAKE

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

  // use tdma's tree scheduler
  tdma_schedule_method_set(TDMA_SCHED_TREE);

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
    uint8_t rssi;
    uint8_t length;
    uint16_t slot;
    uint8_t cnt = 0;

    tdma_init(10);

    while(!tdma_started())
        nrk_wait_until_next_period();

    tdma_schedule_print();

    my_addr8 = tdma_mac_get();
    //my_addr8 = 1;
    my_level = tdma_tree_level_get();

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



/*

void rx_task ()
{
  uint8_t i, len;
  int8_t rssi, val;
  uint8_t *local_rx_buf;
  nrk_time_t check_period;
  printf ("rx_task PID=%d\r\n", nrk_get_pid ());

  // init tdma on channel 25 
  tdma_init (15);

  // Enable AES 128 bit encryption
  // When encryption is active, messages from plaintext
  // source will still be received. 
  //tdma_encryption_set_key(aes_key,16);
  //tdma_encryption_enable();
  // tdma_encryption_disable();

  // By default the RX check rate is 100ms
  // below shows how to change that
  //check_period.secs=0;
  //check_period.nano_secs=200*NANOS_PER_MS;
  //val=tdma_set_rx_check_rate(check_period);

  // The default Clear Channel Assement RSSI threshold is -45
  // Setting this value higher means that you will only trigger
  // receive with a very strong signal.  Setting this lower means
  // tdma will try to receive fainter packets.  If the value is set
  // too high or too low performance will suffer greatly.
  // tdma_set_cca_thresh(-45); 


  //if(val==NRK_ERROR) nrk_kprintf( PSTR("ERROR setting tdma rate\r\n" ));
  // This sets the next RX buffer.
  // This can be called at anytime before releaseing the packet
  // if you wish to do a zero-copy buffer switch
  //tdma_rx_pkt_set_buffer (rx_buf, RF_MAX_PAYLOAD_SIZE);

  while (1) {
    // Wait until an RX packet is received
    val = tdma_wait_until_rx_pkt ();
    // Get the RX packet 
    nrk_led_set (ORANGE_LED);
    local_rx_buf = tdma_rx_pkt_get (&len, &rssi);
    if( tdma_rx_pkt_is_encrypted()==1 ) nrk_kprintf( PSTR( "Packet Encrypted\r\n" ));
    printf ("Got RX packet len=%d RSSI=%d [", len, rssi);
    for (i = 0; i < len; i++)
      printf ("%c", rx_buf[i]);
    printf ("]\r\n");
    nrk_led_clr (ORANGE_LED);
    // Release the RX buffer so future packets can arrive 
    tdma_rx_pkt_release ();
  }

}

uint8_t ctr_cnt[4];

void tx_task ()
{
  uint8_t j, i, val, len, cnt;
  int8_t v;
  nrk_sig_t tx_done_signal;
  nrk_sig_mask_t ret;
  nrk_time_t r_period;

  printf ("tx_task PID=%d\r\n", nrk_get_pid ());

  // Wait until the tx_task starts up tdma
  // This should be called by all tasks using tdma that
  // do not call tdma_init()...
  while (!tdma_started ())
    nrk_wait_until_next_period ();


  // Sample of using Reservations on TX packets
  // This example allows 2 packets to be sent every 5 seconds
  // r_period.secs=5;
  // r_period.nano_secs=0;
  // v=tdma_tx_reserve_set( &r_period, 2 );
  // if(v==NRK_ERROR) nrk_kprintf( PSTR("Error setting b-mac tx reservation (is NRK_MAX_RESERVES defined?)\r\n" ));


  // Get and register the tx_done_signal if you want to
  // do non-blocking transmits
  tx_done_signal = tdma_get_tx_done_signal ();
  nrk_signal_register (tx_done_signal);

  ctr_cnt[0]=0; ctr_cnt[1]=0; ctr_cnt[2]=0; ctr_cnt[3]=0;
  cnt = 0;
  while (1) {
    // Build a TX packet
    sprintf (tx_buf, "This is a test %d", cnt);
    nrk_led_set (BLUE_LED);

    // Auto ACK is an energy efficient link layer ACK on packets
    // If Auto ACK is enabled, then tdma_tx_pkt() will return failure
    // if no ACK was received. In a broadcast domain, the ACK's will
    // typically collide.  To avoid this, one can use address decoding. 
    // The functions are as follows:
    // tdma_auto_ack_enable();
    // tdma_auto_ack_disable();

    // Address decoding is a way of preventing the radio from receiving
    // packets that are not address to a particular node.  This will 
    // supress ACK packets from nodes that should not automatically ACK.
    // The functions are as follows:
    // tdma_addr_decode_set_my_mac(uint16_t MAC_ADDR); 
    // tdma_addr_decode_dest_mac(uint16_t DST_ADDR);  // 0xFFFF is broadcast
    // tdma_addr_decode_enable();
    // tdma_addr_decode_disable();

     ctr_cnt[0]=cnt; 
     if(ctr_cnt[0]==255) ctr_cnt[1]++; 
     if(ctr_cnt[1]==255) ctr_cnt[2]++; 
     if(ctr_cnt[2]==255) ctr_cnt[3]++; 
     // You need to increase the ctr on each packet to make the 
     // stream cipher not repeat.
     tdma_encryption_set_ctr_counter(&ctr_cnt,4);

    // For blocking transmits, use the following function call.
    // For this there is no need to register  
     val=tdma_tx_pkt(tx_buf, strlen(tx_buf));
     if(val==NRK_OK) cnt++;
     else nrk_kprintf( PSTR( "NO ack or Reserve Violated!\r\n" ));


    // This function shows how to transmit packets in a
    // non-blocking manner  
    // val = tdma_tx_pkt_nonblocking(tx_buf, strlen (tx_buf));
    // nrk_kprintf (PSTR ("Tx packet enqueued\r\n"));
    // This functions waits on the tx_done_signal
    // ret = nrk_event_wait (SIG(tx_done_signal));

    // Just check to be sure signal is okay
    // if(ret & SIG(tx_done_signal) == 0 ) 
    // nrk_kprintf (PSTR ("TX done signal error\r\n"));
   
    // If you want to see your remaining reservation
    // printf( "reserve=%d ",tdma_tx_reserve_get() );
    
    // Task gets control again after TX complete
    nrk_kprintf (PSTR ("Tx task sent data!\r\n"));
    nrk_led_clr (BLUE_LED);
    nrk_wait_until_next_period ();
  }

}

void nrk_create_taskset ()
{


  RX_TASK.task = rx_task;
  nrk_task_set_stk( &RX_TASK, rx_task_stack, NRK_APP_STACKSIZE);
  RX_TASK.prio = 2;
  RX_TASK.FirstActivation = TRUE;
  RX_TASK.Type = BASIC_TASK;
  RX_TASK.SchType = PREEMPTIVE;
  RX_TASK.period.secs = 1;
  RX_TASK.period.nano_secs = 0;
  RX_TASK.cpu_reserve.secs = 1;
  RX_TASK.cpu_reserve.nano_secs = 500 * NANOS_PER_MS;
  RX_TASK.offset.secs = 0;
  RX_TASK.offset.nano_secs = 0;
  nrk_activate_task (&RX_TASK);



  printf ("Create done\r\n");
}
*/
