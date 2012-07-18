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

#define TDMA_TREEMAKE

#include <nrk.h>
#include <include.h>
#include <ulib.h>
#include <stdio.h>
#include <avr/sleep.h>
#include <hal.h>
#include <tdma_asap.h>
#include <tdma_asap_scheduler.h>
#include <nrk_error.h>
#include <tdma_asap_stats.h>
#include <tdma_asap_tree.h>


nrk_sig_t tdma_data_send_sig;

//#include "main.h"
#define TDMA_CHANNEL 18
//#define TDMA_CCA -48
#define TDMA_CCA -25

// Only require MAC address for address decode 
#define MAC_ADDR    0x0001
#define EVENT_PERIOD_SECS 0
#define EVENT_PERIOD_NANOS 300000000
#define EVENT_PERIOD_MULT 1

// eliminate any event creation
// (this just kills off the event task as soon as it does tdma_init())
//#define NO_EVENTS

#define TDMA_MODE TDMA_SLAVE
#define SMALL_STACK 256

#define STATS_STACK 512


#define PKT_QUEUE

#ifdef PKT_QUEUE



struct test_ev
{
    uint8_t sender;
    uint16_t seq;
    uint8_t lvl;
    nrk_time_t ts; // timestamp
    nrk_time_t clat; // cumulative latency
};

#define EVPKT_SIZE (24+TDMA_DATA_START)
//#define EVENT_QSIZE (RF_MAX_PAYLOAD_SIZE / EVPKT_SIZE)
#define EVENT_QSIZE 11

// the packet queue is a circular buffer
struct test_ev ev_queue[EVENT_QSIZE];
uint8_t pkt_tx_buf[EVPKT_SIZE];
uint8_t end, begin, fill_cnt;

#else
uint8_t test_tx_buf[2][RF_MAX_PAYLOAD_SIZE];
uint8_t test_tx_offset[2];
uint8_t current_buf;

uint8_t test_rx_offset;

#endif // ifdef PKT_QUEUE


nrk_time_t delay_time;
nrk_time_t stats_delay_time;
nrk_time_t tx_delay_time;
nrk_time_t event_delay_time;

NRK_STK tx_task_stack[SMALL_STACK];
nrk_task_type txTask;

NRK_STK rx_task_stack[SMALL_STACK];
nrk_task_type rxTask;

NRK_STK ev_task_stack[NRK_APP_STACKSIZE];
nrk_task_type evTask;

NRK_STK stats_task_stack[STATS_STACK];
nrk_task_type statsTask;

void nrk_create_taskset ();

uint8_t rx_buf[RF_MAX_PAYLOAD_SIZE];

nrk_time_t current_time;

uint8_t my_addr8;
uint8_t my_level;
uint8_t have_data_to_send;
uint16_t pkt_num;

#ifdef PKT_QUEUE

int8_t gen_event(uint8_t addr, uint16_t event_nr, uint8_t level, uint8_t ev_type)
{
    nrk_time_get(&current_time);
    nrk_time_compact_nanos(&current_time);

    if (fill_cnt >= EVENT_QSIZE)
    {
        printf("DROP %d %d %d\r\n", addr, event_nr, level);
        return NRK_ERROR;
    }
    else if (ev_type == 1)
    {
        printf("GEN %d %d %d %lu %lu\r\n", 
            my_addr8, event_nr, my_level,
            current_time.secs, current_time.nano_secs);
    }
    
    ev_queue[end].sender = my_addr8;
    ev_queue[end].seq = event_nr;
    ev_queue[end].lvl = level;
    ev_queue[end].ts = current_time;
    ev_queue[end].clat.secs = 0;
    ev_queue[end].clat.nano_secs = 0;

    end=(end+1) % EVENT_QSIZE;
    fill_cnt++;
    nrk_event_signal(tdma_data_send_sig);
}


/** Event packet spec
**  0: CTL
**  1: Packet sender addr
**  2: Packet number (hi)
**  3: Packet number (lo)
**  4: Event sender addr
**  5: Event number (hi)
**  6: Event number (lo)
**  7: Event level ()
*/
int8_t fill_txbuf(uint8_t * tx_buf, struct test_ev event)
{
    nrk_time_t t1,t2;
    t1.secs = 0; t1.nano_secs=0;
    t2.secs = 0; t2.nano_secs=0;

    tx_buf[TDMA_DATA_START  ]=0xF5;
    tx_buf[TDMA_DATA_START+1]=my_addr8;
    tx_buf[TDMA_DATA_START+2]=pkt_num>>8;
    tx_buf[TDMA_DATA_START+3]=pkt_num&0xFF;

    tx_buf[TDMA_DATA_START+4]=event.sender;
    tx_buf[TDMA_DATA_START+5]=event.seq>>8;
    tx_buf[TDMA_DATA_START+6]=event.seq&0xFF;
    tx_buf[TDMA_DATA_START+7]=event.lvl;
    
    // put the cumulative latency in the packet 
    memcpy(&tx_buf[TDMA_DATA_START+8], &event.clat, sizeof(event.clat));

    // also put the timestamp of the arrival of the packet so that the MAC 
    // layer can figure out how long it spent at this node
    memcpy(&tx_buf[TDMA_DATA_START+16], &event.ts, sizeof(event.ts));

// testing stuff
    //memcpy(&lat, &tx_buf[TDMA_DATA_START+8],  sizeof(lat));
    //memcpy(&t1, &tx_buf[3+8],  sizeof(t1));

    //memcpy(&old, &tx_buf[TDMA_DATA_START+16],  sizeof(old));
    //memcpy(&t2, &tx_buf[3+16],  sizeof(t2));

//end testing

    //printf("EVTS %lu %lu %lu %lu\r\n", event.clat.secs, event.clat.nano_secs, event.ts.secs, event.ts.nano_secs);
    //printf("RDTS %lu %lu %lu %lu\r\n", t1.secs, t1.nano_secs, t2.secs, t2.nano_secs);

    fill_cnt--;
    begin= (begin+1) % EVENT_QSIZE;

    return EVPKT_SIZE; // length of packet
}

int8_t rte_event(uint8_t * ev_buf)
{
    uint16_t pkt_nbr;

    pkt_nbr = ev_buf[TDMA_DATA_START+2];
    pkt_nbr <<= 8;
    pkt_nbr |= ev_buf[TDMA_DATA_START+3];

    printf("PKTGET %d %d %d\r\n", 
        ev_buf[TDMA_DATA_START+1],
        pkt_nbr,
        my_addr8);

    ev_queue[end].sender = ev_buf[TDMA_DATA_START+4];
    ev_queue[end].seq  =   ev_buf[TDMA_DATA_START+5]<<8;
    ev_queue[end].seq |=   ev_buf[TDMA_DATA_START+6]&0xFF;
    ev_queue[end].lvl  =   ev_buf[TDMA_DATA_START+7];

    // get the latency that was sent in the packet
    memcpy(&ev_queue[end].clat, &ev_buf[TDMA_DATA_START+8], sizeof(ev_queue[end].clat));

    printf("RT %d %d %lu %lu\r\n", ev_queue[end].sender, ev_queue[end].seq, ev_queue[end].clat.secs, ev_queue[end].clat.nano_secs);

    // timestamp the arrival of the packet
    nrk_time_get(&ev_queue[end].ts);

    end=(end+1) % EVENT_QSIZE;
    fill_cnt++;
    nrk_event_signal(tdma_data_send_sig);

    return NRK_OK;
}

// root gets event
int8_t get_event(uint8_t * ev_buf)
{
    struct test_ev cur;
    uint16_t pkt_nbr;

    nrk_time_get(&current_time);
    nrk_time_compact_nanos(&current_time);

    cur.sender = ev_buf[TDMA_DATA_START+4];
    cur.seq  = ev_buf[TDMA_DATA_START+5];
    cur.seq <<= 8;
    cur.seq |= ev_buf[TDMA_DATA_START+6];
    
    cur.lvl  = ev_buf[TDMA_DATA_START+7];

    pkt_nbr = ev_buf[TDMA_DATA_START+2];
    pkt_nbr <<= 8;
    pkt_nbr |= ev_buf[TDMA_DATA_START+3];

    memcpy(&cur.clat, &ev_buf[TDMA_DATA_START+8], sizeof(cur.clat));

    printf("PKTGET %d %d %d\r\n", 
        ev_buf[TDMA_DATA_START+1],
        pkt_nbr,
        my_addr8);

    printf("RCV %d %d %d %lu %lu \r\n",
        cur.sender, cur.seq, cur.lvl,
        // XXX CHANGED! THIS IS CURRENT TIME or LATENCY
        //current_time.secs, current_time.nano_secs);
        cur.clat.secs, cur.clat.nano_secs);


        //ev_rx_buf[test_rx_offset  ], // address of sender
        //event_num, // event number
        //ev_rx_buf[test_rx_offset+3], // sender's level
        //local_rx_buf[TDMA_DATA_START],   // its token
        //current_time.secs, current_time.nano_secs);
    return NRK_OK;
}

#else

/*
 * These were used for packing
 */
/*
// creates or aggregates current event packet
int8_t add_event(uint8_t addr, uint16_t event_nr, uint8_t level, uint8_t ev_type)
{
    // byte 0: control
    // byte 1: # events in pkt
    /////////////////////////////
    // byte 2: addr
    // byte 3: event number (hi)
    // byte 4: event number (low)
    // byte 5: level
    // repeats

    if ((test_tx_offset[current_buf] + 4) >= RF_MAX_PAYLOAD_SIZE)
    {
        printf("DROP %d %d %d\r\n", addr, event_nr, level);
        return NRK_ERROR;
    }
    else if (ev_type == 1)
    {
        printf("GEN %d %d %d %20"PRIu32" %20"PRIu32"\r\n", 
            my_addr8, event_nr, my_level,
            current_time.secs, current_time.nano_secs);
    }
    
    if (test_tx_offset[current_buf] == TDMA_DATA_START)
    {
        test_tx_buf[current_buf][TDMA_DATA_START] = 0xF5;
        test_tx_buf[current_buf][TDMA_DATA_START+1] = 0;

        // packet's ID: address of sender and number of pkt
        test_tx_buf[current_buf][TDMA_DATA_START+2] = my_addr8;
        test_tx_buf[current_buf][TDMA_DATA_START+3] = pkt_num>>8;
        test_tx_buf[current_buf][TDMA_DATA_START+4] = pkt_num& 0xFF;
        test_tx_offset[current_buf]=TDMA_DATA_START+5;
    }


    test_tx_buf[current_buf][test_tx_offset[current_buf]  ] = addr; 
    test_tx_buf[current_buf][test_tx_offset[current_buf]+1] = event_nr>>8;
    test_tx_buf[current_buf][test_tx_offset[current_buf]+2] = event_nr & 0xFF;
    test_tx_buf[current_buf][test_tx_offset[current_buf]+3] = level;

    test_tx_offset[current_buf]+=4;

    // add 1 to # events in the current packet
    test_tx_buf[current_buf][TDMA_DATA_START+1]++;

    nrk_time_get(&current_time);
    nrk_time_compact_nanos(&current_time);
    //nrk_time_sub(&current_time, current_time, tdma_sync_time_get());
    //nrk_time_compact_nanos(&current_time);
    
    //tdma_tx_pkt(test_tx_buf, TDMA_DATA_START+5);
    
    have_data_to_send = 1;

    return NRK_OK;
}

void tdma_test_get_event(uint8_t * ev_rx_buf)
{
    // event packet: get time of rcv
    nrk_time_get(&current_time);
    nrk_time_compact_nanos(&current_time);
    //nrk_time_sub(&current_time, current_time, tdma_sync_time_get());
    //nrk_time_compact_nanos(&current_time);

    // parse the packet


    uint16_t rcv_pktnum = ev_rx_buf[TDMA_DATA_START+3];
    rcv_pktnum <<= 8;
    rcv_pktnum |= ev_rx_buf[TDMA_DATA_START+4];

    // CHANGE 01/05/10 print my mac and timestamp at the end
    //printf("PKTGET %d %d %d %20"PRIu32" %20"PRIu32"\r\n", ev_rx_buf[TDMA_DATA_START+2],rcv_pktnum,
    //     tdma_mac_get(), current_time.secs, current_time.nano_secs );

    printf("PKTGET %d %d %d\r\n", ev_rx_buf[TDMA_DATA_START+2],rcv_pktnum,
         tdma_mac_get());
    test_rx_offset = TDMA_DATA_START+5;

    //printf("GET: %d entries\r\n", ev_rx_buf[TDMA_DATA_START+1]);
    for (uint8_t i = 0; i < ev_rx_buf[TDMA_DATA_START+1]; i++)
    {
        uint16_t event_num = 0;
        event_num = ev_rx_buf[test_rx_offset+1];
        event_num <<= 8;
        event_num |= ev_rx_buf[test_rx_offset+2];

        printf("RCV %d %d %d %20"PRIu32" %20"PRIu32"\r\n",
            ev_rx_buf[test_rx_offset  ], // address of sender
            event_num, // event number
            ev_rx_buf[test_rx_offset+3], // sender's level
            //local_rx_buf[TDMA_DATA_START],   // its toke//n
            current_time.secs, current_time.nano_secs);

        test_rx_offset += 4;
    }

}

void tdma_test_route_event(uint8_t * ev_rx_buf)
{
       // event packet 
    nrk_time_get(&current_time);
    nrk_time_compact_nanos(&current_time);
    //nrk_time_sub(&current_time, current_time, tdma_sync_time_get());
    //nrk_time_compact_nanos(&current_time);

//  printf("RTE %d %d %d %d %20"PRIu32" %20"PRIu32"\r\n",
//      tdma_mac_get(),
//      ev_rx_buf[TDMA_DATA_START+1], // address of sender
//      ev_rx_buf[TDMA_DATA_START+2], // event number
//      ev_rx_buf[TDMA_DATA_START+3], // sender's level
//      local_rx_buf[TDMA_DATA_START],   // its token
//      current_time.secs, current_time.nano_secs);

    // parse the packet
    uint16_t rcv_pktnum = ev_rx_buf[TDMA_DATA_START+3];
    rcv_pktnum <<= 8;
    rcv_pktnum |= ev_rx_buf[TDMA_DATA_START+4];

    //printf("PKTGET %d %d %d %20"PRIu32" %20"PRIu32"\r\n", ev_rx_buf[TDMA_DATA_START+2],rcv_pktnum,
            //tdma_mac_get(), current_time.secs, current_time.nano_secs);
    printf("PKTGET %d %d %d\r\n", ev_rx_buf[TDMA_DATA_START+2],rcv_pktnum,
            tdma_mac_get());

    test_rx_offset = TDMA_DATA_START+5;

    //printf("ROUTE: %d entries\r\n", ev_rx_buf[TDMA_DATA_START+1]);
    for (uint8_t i = 0; i < ev_rx_buf[TDMA_DATA_START+1]; i++)
    {
        uint16_t event_num = ev_rx_buf[test_rx_offset+1];
        event_num <<= 8;
        event_num |= ev_rx_buf[test_rx_offset+2];

        add_event(ev_rx_buf[test_rx_offset  ],
                  event_num,
                  ev_rx_buf[test_rx_offset+3], 0);

        //printf("RTE %d %d %d %d\r\n", ev_rx_buf[test_rx_offset],
        //                           event_num,
        //                           ev_rx_buf[test_rx_offset+3],
        //                           test_rx_offset);
//        printf("RCV %d %d %d %20"PRIu32" %20"PRIu32"\r\n",
//            ev_rx_buf[test_rx_offset  ], // address of sender
//            ev_rx_buf[test_rx_offset+1], // event number
//            ev_rx_buf[test_rx_offset+2], // sender's level
//            //local_rx_buf[TDMA_DATA_START],   // its token
//            current_time.secs, current_time.nano_secs);

        test_rx_offset += 4;
    }
}
*/


#endif

int main ()
{

  nrk_setup_ports ();
  nrk_setup_uart (UART_BAUDRATE_115K2);

  nrk_init ();

  nrk_led_clr (0);
  nrk_led_clr (1);
  nrk_led_clr (2);
  nrk_led_clr (3);

  // TO ACCOUNT FOR TRIALS WHEN SLEEPING IS DONE FOR 4 SECS 
  // BETWEEN SLAVE AND MASTER BOOT
  /*
  if (TDMA_MODE == TDMA_MASTER)
    nrk_time_set (4, 0);
  else
    nrk_time_set (0, 0);
  */

  nrk_time_set(0, 0);

  tdma_mode_set(TDMA_MODE);

  //specify to use TDMA's tree builder
  //tdma_schedule_method_set(TDMA_SCHED_TREE);

  /********** USE MANUAL SCHED**************/


  tdma_schedule_clr();
  tdma_schedule_method_set(TDMA_SCHED_MANUAL);
  
  
/*
  if (TDMA_MODE==TDMA_MASTER)
  {
      tdma_schedule_add(5, TDMA_TXSYNC, 0);
      //tdma_schedule_add(10, TDMA_RX, -1);
      tdma_schedule_add(10, TDMA_RXDATA, 0);
  }
  else
  {
      tdma_schedule_add(5, TDMA_RXSYNC, 0);
      //tdma_schedule_add(10, TDMA_TX_PARENT, 1);
      tdma_schedule_add(10, TDMA_TXDATA, 0);
  }
*/

    if (TDMA_MODE==TDMA_MASTER)
    {
        tdma_schedule_add(49, TDMA_RXDATA, -1);
        tdma_schedule_add(99, TDMA_RXDATA, -1);
        tdma_schedule_add(25, TDMA_TXSYNC, 0);
    }
    // same priorities to every slot
    else if (tdma_mac_get() == 2)
    {
        tdma_schedule_add(49, TDMA_TXDATA, 0);
        tdma_schedule_add(99, TDMA_TXDATA, 0);
        tdma_schedule_add(25, TDMA_RXSYNC, -1);
    }
    else if (tdma_mac_get() == 3)
    {
        tdma_schedule_add(49, TDMA_TXDATA, 1);
        tdma_schedule_add(99, TDMA_TXDATA, 1);
        tdma_schedule_add(25, TDMA_RXSYNC, -1);
    }
    else if (tdma_mac_get() == 4)
    {
        tdma_schedule_add(49, TDMA_TXDATA, 2);
        tdma_schedule_add(99, TDMA_TXDATA, 2);
        tdma_schedule_add(25, TDMA_RXSYNC, -1);
    }
    else if (tdma_mac_get() == 5)
    {
        tdma_schedule_add(49, TDMA_TXDATA, 3);
        tdma_schedule_add(99, TDMA_TXDATA, 3);
        tdma_schedule_add(25, TDMA_RXSYNC, -1);
    }
    else if (tdma_mac_get() == 6)
    {
        tdma_schedule_add(49, TDMA_TXDATA, 4);
        tdma_schedule_add(99, TDMA_TXDATA, 4);
        tdma_schedule_add(25, TDMA_RXSYNC, -1);
    }
    // alternating priorities
    else if (tdma_mac_get() == 7)
    {
        tdma_schedule_add(49, TDMA_TXDATA, 0);
        tdma_schedule_add(99, TDMA_TXDATA, 1);
        tdma_schedule_add(25, TDMA_RXSYNC, -1);
    }
    else if (tdma_mac_get() == 8)
    {
        tdma_schedule_add(49, TDMA_TXDATA, 1);
        tdma_schedule_add(99, TDMA_TXDATA, 0);
        tdma_schedule_add(25, TDMA_RXSYNC, -1);
    }
  
//      tdma_schedule_add(0, TDMA_TX_CHILD, 0);
//      tdma_schedule_add(15, TDMA_TX_CHILD, 0);
      //tdma_schedule_add(15, TDMA_RX, -1);

  // an alternate way of doing things
/*
  if (TDMA_MODE==TDMA_MASTER)
  {
	// use the macro from testslotstride.h
	//SLOTSCHED
    

    //tdma_schedule_add(0, TDMA_TX_CHILD, 0);
    //tdma_schedule_add(20, TDMA_TX_CHILD, -1);
    //tdma_schedule_add(40, TDMA_TX_CHILD, 0);
    //tdma_schedule_add(60, TDMA_TX_CHILD, 0);
    //tdma_schedule_add(80, TDMA_TX_CHILD, 0);
#if 0
      tdma_schedule_add(8, TDMA_TX_CHILD, 0);
      tdma_schedule_add(2, TDMA_RX, -1);
      //tdma_schedule_add(10, TDMA_TX_CHILD, 0);
#endif


  }
  else
  {
      tdma_schedule_add(8, TDMA_RX, -1);
      tdma_schedule_add(0, TDMA_TX_CHILD, 0);
      tdma_schedule_add(2, TDMA_TX_PARENT, 0);
  }
*/
   
  
  /* this wasn't used anyway
      tdma_schedule_add(20, TDMA_TX_CHILD, 0);
      tdma_schedule_add(25, TDMA_TX_CHILD, 0);
      tdma_schedule_add(30, TDMA_TX_CHILD, 0);
      tdma_schedule_add(35, TDMA_TX_CHILD, 0);
      tdma_schedule_add(40, TDMA_TX_CHILD, 0);
  */
  /************END MAUAL SCHED ****************/

  // avoiding using periods -- have a 'delay time' instead
  delay_time.secs = 1;
  delay_time.nano_secs = 0;

  stats_delay_time.secs = 10;
  stats_delay_time.nano_secs = 0;
  
  tdma_task_config ();

  nrk_create_taskset ();

#ifndef PKT_QUEUE
  current_buf = 0;
  test_tx_offset[0] = TDMA_DATA_START;
  test_tx_offset[1] = TDMA_DATA_START;

#endif
  nrk_start ();

  return 0;
}

// generate events
void event_task()
{
    uint8_t r;
    uint16_t cnt;
    nrk_time_t offset_time;

    printf("evTask %d\r\n", nrk_get_pid());

    r=0;
    cnt = 0;

    event_delay_time.secs = EVENT_PERIOD_SECS;
    event_delay_time.nano_secs = EVENT_PERIOD_NANOS;

    tdma_data_send_sig = nrk_signal_create();

    tdma_set_cca_thresh(TDMA_CCA);
    tdma_init(TDMA_CHANNEL);


    while (!tdma_started())
        nrk_wait(delay_time);
    
    //tdma_schedule_print();

    //while (1)
    //    nrk_wait(delay_time);
    //PORTA |= BM(DEBUG_1);
    // pretend NOT
    /*
    while(1)
    {
    PORTA &= ~BM(DEBUG_1);
        nrk_wait_until_next_period();
    PORTA |= BM(DEBUG_1);

    }
    */

    my_addr8 = tdma_mac_get() & 0xFF;
    my_level = tdma_tree_level_get();

    //nrk_kprintf(PSTR("Setting AUTOACK\r\n"));
    //tdma_auto_ack_enable();
    //tdma_addr_decode_set_my_mac(tdma_mac_get()); 
    //tdma_addr_decode_dest_mac(tdma_tree_parent_get());  // 0xFFFF is broadcast
    //tdma_addr_decode_enable();

    //nrk_kprintf(PSTR("EVTASK Start\r\n"));
    printf("NODE %d %d %d\r\n", tdma_mac_get(), tdma_tree_level_get(), tdma_tree_parent_get());

    if (TDMA_MODE == TDMA_MASTER)
        nrk_terminate_task();

    #ifdef NO_EVENTS
        nrk_terminate_task();
    #endif
    // wait for random amount of time to offset
    srand(my_addr8);
    offset_time.secs=0;
    offset_time.nano_secs = rand();
    nrk_time_compact_nanos(&offset_time);

    nrk_wait(offset_time);

    while(1)
    {
        //if (my_addr16 != 3 || (cnt % 3 == 0))
        //r = rand() % 2;
        //r = rand() % 100;
        r= (r+1) % (EVENT_PERIOD_MULT) ;
        //printf("r=%d\r\n",r);

        if ((tdma_my_mode_get()==TDMA_SLAVE) && (r==0))
        {
            // only generate if I am a leaf node
            //if (tdma_tree_is_leaf())
            #ifdef PKT_QUEUE
            gen_event(my_addr8, cnt, my_level, 1);
            #else
            add_event(my_addr8, cnt, my_level, 1);
            #endif
            cnt = (cnt+1) % 65536;
            //  tdma_test_generate_event(cnt++);
            
           // sprintf( &tx_buf[TDMA_DATA_START], "Hello World %d", my_addr16 ); 
           // length=strlen(&tx_buf[TDMA_DATA_START])+TDMA_DATA_START;
           // tdma_tx_pkt( tx_buf, length);
            
        }
        //else
        //    nrk_kprintf(PSTR("No Event\r\n"));
        //nrk_led_clr(RED_LED);
        //PORTA &= ~BM(DEBUG_1);

        //nrk_wait_until_next_period();
        nrk_wait(event_delay_time);
        //event_delay_time.secs+=20;
            //printf("ITERATION %20"PRIu32"\r\n", event_delay_time.secs);

        //PORTA |= BM(DEBUG_1);
    }

}

void tx_task()
{ 

   
    printf("txTask %d\r\n", nrk_get_pid());
    have_data_to_send = 0;

    while(!tdma_started())
        nrk_wait(delay_time);

    tx_delay_time.secs = 0;
    tx_delay_time.nano_secs=500000000;
    //srand(my_addr16);

    //while (1)
    //    nrk_wait(delay_time);
    while(1)
    {

        #ifdef PKT_QUEUE
            while (!fill_cnt)
        #else
            while (!have_data_to_send)
        #endif
        {
            nrk_signal_register(tdma_data_send_sig);
            nrk_event_wait(SIG(tdma_data_send_sig));
            //nrk_wait(tx_delay_time);
            //nrk_wait_until_next_period();
        }

        if( tdma_tx_pkt_check()!=0 )
        {
            nrk_kprintf(PSTR("Pending\r\n"));
            //printf( "Pending on slot %d\r\n",MY_TX_SLOT );
        }
        else
        {
            #ifdef PKT_QUEUE
                fill_txbuf(pkt_tx_buf, ev_queue[begin]);
                // XXX had to change this to reflect sending nrk_time_t in the packet to calc latency
                //tdma_tx_pkt(pkt_tx_buf, EVPKT_SIZE);
                tdma_tx_pkt(pkt_tx_buf, EVPKT_SIZE-sizeof(nrk_time_t));
            #else
                tdma_tx_pkt(test_tx_buf[current_buf], test_tx_offset[current_buf]+1);
            //printf("ttb %s, off %d\r\n", test_tx_buf[current_buf]+TDMA_DATA_START, test_tx_offset[current_buf]);


                // switch buffers
                current_buf = (current_buf + 1) % 2;
                test_tx_offset[current_buf] = TDMA_DATA_START;
                test_tx_buf[current_buf][TDMA_DATA_START] = 0;
                have_data_to_send = 0;
            #endif

            pkt_num++;


            // FOR DEBUGGING.  Every 50 pkts, get a new value for the SFD-SLOT wait.
#ifdef SFD_TEST
            if (pkt_num % 25 == 0)
                sfd_to_slot_time += 100;
#endif
            
        }

        tdma_wait_until_tx();
        //PORTA |= BM(DEBUG_1);
        //PORTA &= ~BM(DEBUG_1);

        // CHANGE 01/05/10 I will send the packet to my parent, the intended recipient, so that is the last field printed here
        nrk_time_get(&current_time);
        nrk_time_compact_nanos(&current_time);

        // need a time associated with it if we want to track the order of the packets
        //printf("PKTSEND %u %u %u %u %20"PRIu32" %20"PRIu32"\r\n", my_addr8, pkt_num-1, my_level, tdma_tree_parent_get(), current_time.secs, current_time.nano_secs);
        printf("PKTSEND %u %u %u %u\r\n", my_addr8, pkt_num-1, my_level, tdma_tree_parent_get());
    }
}



void rx_task()
{
    uint8_t *local_rx_buf;
    int8_t rssi;
    uint8_t length,slot;
    
    printf("rxTask %d\r\n", nrk_get_pid());

    tdma_rx_pkt_set_buffer(rx_buf, RF_MAX_PAYLOAD_SIZE);

    while(!tdma_started())
        nrk_wait(delay_time);

    //while (1)
        //nrk_wait(delay_time);

    while(1)
    {
        if( tdma_rx_pkt_check()!=0 )
        {
            local_rx_buf=tdma_rx_pkt_get(&length, &rssi, &slot);
            //printf( "Got Packet len %d: ",length );

            if (local_rx_buf[TDMA_DATA_START] == 0xF5)
            {
                if (tdma_my_mode_get() == TDMA_SLAVE)
                {
                    #ifdef PKT_QUEUE
                        rte_event(local_rx_buf);
                    #else
                        tdma_test_route_event(local_rx_buf);
                    #endif
                }
                else
                {
                    #ifdef PKT_QUEUE
                        get_event(local_rx_buf);
                    #else
                        tdma_test_get_event(local_rx_buf);
                    #endif
                }
            }

            local_rx_buf[TDMA_DATA_START] = 1;
            tdma_rx_pkt_release();
         //   nrk_led_clr(BLUE_LED);
        }
        tdma_wait_until_rx_pkt();

    /*
        PORTA |= BM(DEBUG_1);
        PORTA &= ~BM(DEBUG_1);
    */
    }
}


void stats_task()
{
    uint8_t r =0;

    printf("statTask %d\r\n", nrk_get_pid());
    //uint8_t mode = 0;

    //stats_delay_time.secs = 10;
    //stats_delay_time.nano_secs = 0;

    while(!tdma_started())
        nrk_wait(delay_time);

    while (1)
    {
        // print out stats every once in a while
        nrk_wait(stats_delay_time);

        // was mod 50
        //r = (r+1) % 10;

        //nrk_wait_until(stats_delay_time);
        //stats_delay_time.secs+=10;
        #ifdef MPKT_LOG
        tdma_stats_mpkt_dump();
        #endif

        if (r == 0)
        {
            tdma_stats_dump();
            nrk_time_get(&current_time);
            // also print the queue size, although this causes an idle/kernel stack overflow
            printf("EVQ %d %d %d %lu %lu\r\n", tdma_mac_get(), tdma_tree_level_get(), fill_cnt,
                    current_time.secs, current_time.nano_secs);
        }
    }
}


void
nrk_create_taskset()
{
  
  evTask.task = event_task;
  //nrk_task_set_stk( &evTask, tx_task_stack, NRK_APP_STACKSIZE);
  evTask.Ptos = (void *) &ev_task_stack[NRK_APP_STACKSIZE-1];
  evTask.Pbos = (void *) &ev_task_stack[0];
  evTask.prio = 2;
  evTask.FirstActivation = TRUE;
  evTask.Type = BASIC_TASK;
  evTask.SchType = PREEMPTIVE;
  //evTask.period.secs = EVENT_PERIOD_SECS;
  //evTask.period.nano_secs = EVENT_PERIOD_NANOS;
  evTask.period.secs = 0;
  evTask.period.nano_secs = 0;
  evTask.cpu_reserve.secs = 0;
  evTask.cpu_reserve.nano_secs = 0;
  evTask.offset.secs = 0;
  evTask.offset.nano_secs = 0;
  nrk_activate_task (&evTask);

  rxTask.task = rx_task;
  rxTask.Ptos = (void *) &rx_task_stack[SMALL_STACK-1];
  rxTask.Pbos = (void *) &rx_task_stack[0];
  rxTask.prio = 3;
  rxTask.FirstActivation = TRUE;
  rxTask.Type = BASIC_TASK;
  rxTask.SchType = PREEMPTIVE;
  rxTask.period.secs = 0;
  rxTask.period.nano_secs = 0;
  rxTask.cpu_reserve.secs = 0;
  rxTask.cpu_reserve.nano_secs = 0;//100*NANOS_PER_MS;
  rxTask.offset.secs = 0;
  rxTask.offset.nano_secs= 0;
  nrk_activate_task (&rxTask);
  
  if (TDMA_MODE==TDMA_SLAVE)
  {
  txTask.task = tx_task;
  //nrk_task_set_stk( &txTask, tx_task_stack, NRK_APP_STACKSIZE);
  txTask.Ptos = (void *) &tx_task_stack[SMALL_STACK-1];
  txTask.Pbos = (void *) &tx_task_stack[0];
  txTask.prio = 4;
  txTask.FirstActivation = TRUE;
  txTask.Type = BASIC_TASK;
  txTask.SchType = PREEMPTIVE;
  //txTask.period.secs = 2;
  txTask.period.secs = 0;
  txTask.period.nano_secs = 0;
  txTask.cpu_reserve.secs = 0;
  txTask.cpu_reserve.nano_secs = 0;
  txTask.offset.secs = 0;
  txTask.offset.nano_secs = 0;
  nrk_activate_task (&txTask);
  //nrk_kprintf ( PSTR("Create done\r\n") );
  }

  statsTask.task = stats_task;
  //nrk_task_set_stk( &statsTask, tx_task_stack, NRK_APP_STACKSIZE);
  statsTask.Ptos = (void *) &stats_task_stack[STATS_STACK-1];
  statsTask.Pbos = (void *) &stats_task_stack[0];
  statsTask.prio = 1;
  statsTask.FirstActivation = TRUE;
  statsTask.Type = BASIC_TASK;
  statsTask.SchType = PREEMPTIVE;
  //statsTask.period.secs = 2;
  statsTask.period.secs = 0;
  statsTask.period.nano_secs = 0;
  statsTask.cpu_reserve.secs = 0;
  statsTask.cpu_reserve.nano_secs = 0;
  statsTask.offset.secs = 0;
  statsTask.offset.nano_secs = 0;
  nrk_activate_task (&statsTask);

}
