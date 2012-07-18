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
#include <bmac.h>
#include <nrk_error.h>
#include <avr/eeprom.h>
#include <nrk_eeprom.h>

nrk_sig_t bmac_data_send_sig;
// Only require MAC address for address decode 
#define MAC_ADDR	0x0001

#define BMAC_SOURCE 1
#define BMAC_SINK 2


//#define DBG_PKT_ORDER

// max number of retries before giving up packet send
//#define BMAC_UNLIMITED_RETRY

#ifndef BMAC_UNLIMITED_RETRY
    #define MAX_RETRIES 5
#endif


// for compatibility
#define BMAC_DATA_START 0
#define STATS_STACK 512

#define NODE_MODE BMAC_SOURCE

#define EVENT_PERIOD_SECS 0
#define EVENT_PERIOD_NANOS 500000000
#define EVENT_PERIOD_MULT 1

nrk_task_type statsTask;
nrk_task_type RX_TASK;
nrk_task_type evTask;
nrk_task_type TX_TASK;

NRK_STK rx_task_stack[STATS_STACK];
NRK_STK tx_task_stack[STATS_STACK];
NRK_STK ev_task_stack[STATS_STACK];
NRK_STK stats_task_stack[STATS_STACK];


struct test_ev
{
    uint8_t sender;
    uint16_t seq;
    uint8_t lvl;
    nrk_time_t ts; // timestamp
    nrk_time_t clat; // cumulative latency
};

#define EVPKT_SIZE (24+BMAC_DATA_START)
#define EVENT_QSIZE (RF_MAX_PAYLOAD_SIZE / EVPKT_SIZE)

// the packet queue is a circular buffer
struct test_ev ev_queue[EVENT_QSIZE];
uint8_t pkt_tx_buf[EVPKT_SIZE];
uint8_t end, begin, fill_cnt;

void stats_task(void);
void rx_task (void);
void tx_task (void);

void nrk_create_taskset ();

uint8_t tx_buf[RF_MAX_PAYLOAD_SIZE];
uint8_t rx_buf[RF_MAX_PAYLOAD_SIZE];
uint8_t aes_key[16]={0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e, 0x0f};

uint8_t test_tx_buf[2][RF_MAX_PAYLOAD_SIZE];
uint8_t test_tx_offset[2];
uint8_t current_buf;

uint8_t test_rx_offset;
nrk_time_t current_time;
nrk_time_t event_delay_time;
nrk_time_t delay_time;
nrk_sig_t tx_done_signal;

uint8_t my_addr8;
uint8_t my_level;
uint8_t my_parent;
uint8_t have_data_to_send;
uint16_t pkt_num;

uint16_t pkt_nbr_expect;
uint16_t errs;


// tree stuff

#define TREE_MAX_CHILDREN 5

typedef struct
{
  uint8_t level;
  uint8_t isDead;
  uint16_t address;
  uint16_t parent;
  uint8_t totalChildren;
  uint16_t children[TREE_MAX_CHILDREN];
} sensorInfo;

#define DIM 17
uint16_t child_addrs[TREE_MAX_CHILDREN];
uint8_t num_children;
sensorInfo sensorsInfo[DIM];

void test_setup()
{
    uint32_t mac_address;
    uint8_t i,j;
    uint8_t hops, tmplv, tmpnode;

    read_eeprom_mac_address(&mac_address);
    my_addr8 = mac_address & 0xFF;

    // set of parents represents the tree.

    sensorsInfo[0 ].isDead = 1; // doesn't exist
    sensorsInfo[1 ].isDead = 0;
        sensorsInfo[1 ].parent = 0; // have no parent
    sensorsInfo[2 ].isDead = 0;
        sensorsInfo[2 ].parent = 1;
    sensorsInfo[3 ].isDead = 0;
        sensorsInfo[3 ].parent = 14;
    sensorsInfo[4 ].isDead = 0;
        sensorsInfo[4 ].parent = 1;
    sensorsInfo[5 ].isDead = 0;
        sensorsInfo[5 ].parent = 14;
    sensorsInfo[6 ].isDead = 0;
        sensorsInfo[6 ].parent = 5;
    sensorsInfo[7 ].isDead = 0;
        sensorsInfo[7 ].parent = 11;
    sensorsInfo[8 ].isDead = 0;
        sensorsInfo[8 ].parent = 1;
    sensorsInfo[9 ].isDead = 0;
        sensorsInfo[9 ].parent = 12;
    sensorsInfo[10].isDead = 0;
        sensorsInfo[10].parent = 8;
    sensorsInfo[11].isDead = 0;
        sensorsInfo[11].parent = 1;
    sensorsInfo[12].isDead = 0;
        sensorsInfo[12].parent = 16;
    sensorsInfo[13].isDead = 0;
        sensorsInfo[13].parent = 10;
    sensorsInfo[14].isDead = 0;
        sensorsInfo[14].parent = 1;
    sensorsInfo[15].isDead = 0;
        sensorsInfo[15].parent = 9;
    sensorsInfo[16].isDead = 0;
        sensorsInfo[16].parent = 6;


    for (i = 0; i < 17; i++)
    {
        sensorsInfo[i].address = i;
        sensorsInfo[i].level = 0xFF;
    }

    // master is at level 0
    sensorsInfo[1].level = 0;

    // fill the children arrays according to the tree structure
    for (i = 1; i < 17; i++)
    {
        if (!sensorsInfo[i].isDead)
        {
            // find the level
            // do this by searching up from my node (following parent
            // links) until I find a node whose level has been determined.
            // then I decide my own level based on how many links I am from them.
            tmplv = sensorsInfo[i].level; // will be 0xFF if not determined yet
            tmpnode = i;
            hops = 0; // # of hops from me to known level node

            while (tmplv == 0xFF)
            {
                hops++;
                tmpnode = sensorsInfo[tmpnode].parent;
                tmplv = sensorsInfo[tmpnode].level;
            }

            sensorsInfo[i].level = hops+tmplv;
        
            // set depth of tree to be maximum level of any node
            //if (depth_of_tree < sensorsInfo[i].level)
            //    depth_of_tree = sensorsInfo[i].level;

            // loop through sensors and add all nodes with children that have me
            // as their parent
            for (j = 1; j < 17; j++)
            {
                if (i!=j)
                {
                    if (sensorsInfo[j].parent == i)
                    {
                        sensorsInfo[i].children[sensorsInfo[i].totalChildren] = j;
                        sensorsInfo[i].totalChildren++;
                    }
                }
            }
        } // if not dead
    }

    // fill in my children array
    for (i = 0; i < sensorsInfo[my_addr8].totalChildren; i++)
    {
        child_addrs[num_children] = sensorsInfo[my_addr8].children[i];
        num_children++;
    }

    // and my level and parent
    my_level  = sensorsInfo[my_addr8].level;
    my_parent = sensorsInfo[my_addr8].parent;

    if (sensorsInfo[my_addr8].isDead)
    {
        nrk_terminate_task();
    }
}

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
        printf("GEN %d %d %d %20"PRIu32" %20"PRIu32"\r\n", 
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
    nrk_event_signal(bmac_data_send_sig);
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
    tx_buf[BMAC_DATA_START  ]=0xF5;
    tx_buf[BMAC_DATA_START+1]=my_addr8;
    tx_buf[BMAC_DATA_START+2]=pkt_num>>8;
    tx_buf[BMAC_DATA_START+3]=pkt_num&0xFF;

    tx_buf[BMAC_DATA_START+4]=event.sender;
    tx_buf[BMAC_DATA_START+5]=event.seq>>8;
    tx_buf[BMAC_DATA_START+6]=event.seq&0xFF;
    tx_buf[BMAC_DATA_START+7]=event.lvl;
   
    // put the cumulative latency in the packet 
    memcpy(&tx_buf[BMAC_DATA_START+8], &event.clat, sizeof(event.clat));

    // also put the timestamp of the arrival of the packet so that the MAC 
    // layer can figure out how long it spent at this node
    memcpy(&tx_buf[BMAC_DATA_START+16], &event.ts, sizeof(event.ts));
    
    fill_cnt--;
    begin= (begin+1) % EVENT_QSIZE;

    return EVPKT_SIZE; // length of packet
}

int8_t rte_event(uint8_t * ev_buf)
{
    uint16_t pkt_nbr;

    pkt_nbr = ev_buf[BMAC_DATA_START+2];
    pkt_nbr <<= 8;
    pkt_nbr |= ev_buf[BMAC_DATA_START+3];

    printf("PKTGET %d %d %d\r\n", 
        ev_buf[BMAC_DATA_START+1],
        pkt_nbr,
        my_addr8);

    ev_queue[end].sender = ev_buf[BMAC_DATA_START+4];
    ev_queue[end].seq  =   ev_buf[BMAC_DATA_START+5]<<8;
    ev_queue[end].seq |=   ev_buf[BMAC_DATA_START+6]&0xFF;
    ev_queue[end].lvl  =   ev_buf[BMAC_DATA_START+7];

    // get the latency that was sent in the packet
    memcpy(&ev_queue[end].clat, &ev_buf[BMAC_DATA_START+8], sizeof(ev_queue[end].clat));

    printf("RT %d %d %lu %lu\r\n", ev_queue[end].sender, ev_queue[end].seq, ev_queue[end].clat.secs, ev_queue[end].clat.nano_secs);

    // timestamp the arrival of the packet
    nrk_time_get(&ev_queue[end].ts);
    end=(end+1) % EVENT_QSIZE;
    fill_cnt++;

    nrk_event_signal(bmac_data_send_sig);

    return NRK_OK;
}

int8_t get_event(uint8_t * ev_buf)
{
    struct test_ev cur;
    uint16_t pkt_nbr;

    nrk_time_get(&current_time);
    nrk_time_compact_nanos(&current_time);

    cur.sender = ev_buf[BMAC_DATA_START+4];
    cur.seq  = ev_buf[BMAC_DATA_START+5];
    cur.seq <<= 8;
    cur.seq |= ev_buf[BMAC_DATA_START+6];
    
    cur.lvl  = ev_buf[BMAC_DATA_START+7];

    pkt_nbr = ev_buf[BMAC_DATA_START+2];
    pkt_nbr <<= 8;
    pkt_nbr |= ev_buf[BMAC_DATA_START+3];

    memcpy(&cur.clat, &ev_buf[BMAC_DATA_START+8], sizeof(cur.clat));

#ifdef DBG_PKT_ORDER
    // XXX REMEMBER TO TAKE THIS OUT
    if (pkt_nbr > pkt_nbr_expect)
        printf("WRONGPKT %d\r\n", errs++);
        
    pkt_nbr_expect = pkt_nbr+1;
#endif

    printf("PKTGET %d %d %d\r\n", 
        ev_buf[BMAC_DATA_START+1],
        pkt_nbr,
        my_addr8);
    printf("RCV %d %d %d %lu %lu\r\n",
        cur.sender, cur.seq, cur.lvl, 
        //current_time.secs, current_time.nano_secs);
        cur.clat.secs, cur.clat.nano_secs);
        //ev_rx_buf[test_rx_offset  ], // address of sender
        //event_num, // event number
        //ev_rx_buf[test_rx_offset+3], // sender's level
        //local_rx_buf[BMAC_DATA_START],   // its token
        //current_time.secs, current_time.nano_secs);
    return NRK_OK;
}

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
    
    if (test_tx_offset[current_buf] == BMAC_DATA_START)
    {
        test_tx_buf[current_buf][BMAC_DATA_START] = 0xF5;
        test_tx_buf[current_buf][BMAC_DATA_START+1] = 0;

        // packet's ID: address of sender and number of pkt
        test_tx_buf[current_buf][BMAC_DATA_START+2] = my_addr8;
        test_tx_buf[current_buf][BMAC_DATA_START+3] = pkt_num>>8;
        test_tx_buf[current_buf][BMAC_DATA_START+4] = pkt_num& 0xFF;
        test_tx_offset[current_buf]=BMAC_DATA_START+5;
    }


    test_tx_buf[current_buf][test_tx_offset[current_buf]  ] = addr; 
    test_tx_buf[current_buf][test_tx_offset[current_buf]+1] = event_nr>>8;
    test_tx_buf[current_buf][test_tx_offset[current_buf]+2] = event_nr & 0xFF;
    test_tx_buf[current_buf][test_tx_offset[current_buf]+3] = level;

    test_tx_offset[current_buf]+=4;

    // add 1 to # events in the current packet
    test_tx_buf[current_buf][BMAC_DATA_START+1]++;

    nrk_time_get(&current_time);
    nrk_time_compact_nanos(&current_time);
    //nrk_time_sub(&current_time, current_time, tdma_sync_time_get());
    //nrk_time_compact_nanos(&current_time);
    
    //tdma_tx_pkt(test_tx_buf, BMAC_DATA_START+5);
    
    have_data_to_send = 1;
    nrk_led_set(GREEN_LED);

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


    uint16_t rcv_pktnum = ev_rx_buf[BMAC_DATA_START+3];
    rcv_pktnum <<= 8;
    rcv_pktnum |= ev_rx_buf[BMAC_DATA_START+4];

    // CHANGE 01/05/10 print my mac and timestamp at the end
    printf("PKTGET %d %d %d %20"PRIu32" %20"PRIu32"\r\n", ev_rx_buf[BMAC_DATA_START+2],rcv_pktnum,
         my_addr8, current_time.secs, current_time.nano_secs );
    test_rx_offset = BMAC_DATA_START+5;

    printf("GET: %d entries\r\n", ev_rx_buf[BMAC_DATA_START+1]);
    for (uint8_t i = 0; i < ev_rx_buf[BMAC_DATA_START+1]; i++)
    {
        uint16_t event_num = 0;
        event_num = ev_rx_buf[test_rx_offset+1];
        event_num <<= 8;
        event_num |= ev_rx_buf[test_rx_offset+2];

        printf("RCV %d %d %d %20"PRIu32" %20"PRIu32"\r\n",
            ev_rx_buf[test_rx_offset  ], // address of sender
            event_num, // event number
            ev_rx_buf[test_rx_offset+3], // sender's level
            //local_rx_buf[BMAC_DATA_START],   // its token
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

    //printf("RTE %d %d %d %d %20"PRIu32" %20"PRIu32"\r\n",
    //    tdma_mac_get(),
    //    ev_rx_buf[BMAC_DATA_START+1], // address of sender
    //    ev_rx_buf[BMAC_DATA_START+2], // event number
    //    ev_rx_buf[BMAC_DATA_START+3], // sender's level
        //local_rx_buf[BMAC_DATA_START],   // its token
    //    current_time.secs, current_time.nano_secs);

    // parse the packet
    uint16_t rcv_pktnum = ev_rx_buf[BMAC_DATA_START+3];
    rcv_pktnum <<= 8;
    rcv_pktnum |= ev_rx_buf[BMAC_DATA_START+4];

    printf("PKTGET %d %d %d %20"PRIu32" %20"PRIu32"\r\n", ev_rx_buf[BMAC_DATA_START+2],rcv_pktnum,
            my_addr8, current_time.secs, current_time.nano_secs);

    test_rx_offset = BMAC_DATA_START+5;

    printf("ROUTE: %d entries\r\n", ev_rx_buf[BMAC_DATA_START+1]);
    for (uint8_t i = 0; i < ev_rx_buf[BMAC_DATA_START+1]; i++)
    {
        uint16_t event_num = ev_rx_buf[test_rx_offset+1];
        event_num <<= 8;
        event_num |= ev_rx_buf[test_rx_offset+2];

        add_event(ev_rx_buf[test_rx_offset  ],
                  event_num,
                  ev_rx_buf[test_rx_offset+3], 0);

        printf("RTE %d %d %d %d\r\n", ev_rx_buf[test_rx_offset],
                                   event_num,
                                   ev_rx_buf[test_rx_offset+3],
                                   test_rx_offset);
//        printf("RCV %d %d %d %20"PRIu32" %20"PRIu32"\r\n",
//            ev_rx_buf[test_rx_offset  ], // address of sender
//            ev_rx_buf[test_rx_offset+1], // event number
//            ev_rx_buf[test_rx_offset+2], // sender's level
//            //local_rx_buf[BMAC_DATA_START],   // its token
//            current_time.secs, current_time.nano_secs);

        test_rx_offset += 4;
    }
}

int main ()
{
  uint16_t div;
  nrk_setup_ports ();
  nrk_setup_uart (UART_BAUDRATE_115K2);

  nrk_init ();

  nrk_led_clr (0);
  nrk_led_clr (1);
  nrk_led_clr (2);
  nrk_led_clr (3);

  nrk_time_set (0, 0);

  delay_time.secs = 1;
  delay_time.nano_secs = 0;
  bmac_task_config ();

  nrk_create_taskset ();
  nrk_start ();

  return 0;
}

/*
void rx_task ()
{
  uint8_t i, len;
  int8_t rssi, val;
  uint8_t *local_rx_buf;
  nrk_time_t check_period;
  printf ("rx_task PID=%d\r\n", nrk_get_pid ());

  // init bmac on channel 25 
  bmac_init (18);

  // Enable AES 128 bit encryption
  // When encryption is active, messages from plaintext
  // source will still be received. 
  //bmac_encryption_set_key(aes_key,16);
  //bmac_encryption_enable();
  // bmac_encryption_disable();

  // By default the RX check rate is 100ms
  // below shows how to change that
  //check_period.secs=0;
  //check_period.nano_secs=200*NANOS_PER_MS;
  //val=bmac_set_rx_check_rate(check_period);

  // The default Clear Channel Assement RSSI threshold is -45
  // Setting this value higher means that you will only trigger
  // receive with a very strong signal.  Setting this lower means
  // bmac will try to receive fainter packets.  If the value is set
  // too high or too low performance will suffer greatly.
  //bmac_set_cca_thresh(-45); 


  //if(val==NRK_ERROR) nrk_kprintf( PSTR("ERROR setting bmac rate\r\n" ));
  // This sets the next RX buffer.
  // This can be called at anytime before releaseing the packet
  // if you wish to do a zero-copy buffer switch
  bmac_rx_pkt_set_buffer (rx_buf, RF_MAX_PAYLOAD_SIZE);

  while (1) {
    // Wait until an RX packet is received
    val = bmac_wait_until_rx_pkt ();
    // Get the RX packet 
    nrk_led_set (ORANGE_LED);
    local_rx_buf = bmac_rx_pkt_get (&len, &rssi);
    if( bmac_rx_pkt_is_encrypted()==1 ) nrk_kprintf( PSTR( "Packet Encrypted\r\n" ));
    printf ("Got RX packet len=%d RSSI=%d [", len, rssi);
    for (i = 0; i < len; i++)
      printf ("%c", rx_buf[i]);
    printf ("]\r\n");
    nrk_led_clr (ORANGE_LED);
    // Release the RX buffer so future packets can arrive 
    bmac_rx_pkt_release ();
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

  // Wait until the tx_task starts up bmac
  // This should be called by all tasks using bmac that
  // do not call bmac_init()...
  while (!bmac_started ())
    nrk_wait_until_next_period ();


  // Sample of using Reservations on TX packets
  // This example allows 2 packets to be sent every 5 seconds
  // r_period.secs=5;
  // r_period.nano_secs=0;
  // v=bmac_tx_reserve_set( &r_period, 2 );
  // if(v==NRK_ERROR) nrk_kprintf( PSTR("Error setting b-mac tx reservation (is NRK_MAX_RESERVES defined?)\r\n" ));


  // Get and register the tx_done_signal if you want to
  // do non-blocking transmits
  tx_done_signal = bmac_get_tx_done_signal ();
  nrk_signal_register (tx_done_signal);

  ctr_cnt[0]=0; ctr_cnt[1]=0; ctr_cnt[2]=0; ctr_cnt[3]=0;
  cnt = 0;
  while (1) {
    // Build a TX packet
    sprintf (tx_buf, "This is a test %d", cnt);
    nrk_led_set (BLUE_LED);

    // Auto ACK is an energy efficient link layer ACK on packets
    // If Auto ACK is enabled, then bmac_tx_pkt() will return failure
    // if no ACK was received. In a broadcast domain, the ACK's will
    // typically collide.  To avoid this, one can use address decoding. 
    // The functions are as follows:
    // bmac_auto_ack_enable();
    // bmac_auto_ack_disable();

    // Address decoding is a way of preventing the radio from receiving
    // packets that are not address to a particular node.  This will 
    // supress ACK packets from nodes that should not automatically ACK.
    // The functions are as follows:
    // bmac_addr_decode_set_my_mac(uint16_t MAC_ADDR); 
    // bmac_addr_decode_dest_mac(uint16_t DST_ADDR);  // 0xFFFF is broadcast
    // bmac_addr_decode_enable();
    // bmac_addr_decode_disable();

     ctr_cnt[0]=cnt; 
     if(ctr_cnt[0]==255) ctr_cnt[1]++; 
     if(ctr_cnt[1]==255) ctr_cnt[2]++; 
     if(ctr_cnt[2]==255) ctr_cnt[3]++; 
     // You need to increase the ctr on each packet to make the 
     // stream cipher not repeat.
     bmac_encryption_set_ctr_counter(&ctr_cnt,4);

    // For blocking transmits, use the following function call.
    // For this there is no need to register  
     val=bmac_tx_pkt(tx_buf, strlen(tx_buf));
     if(val==NRK_OK) cnt++;
     else nrk_kprintf( PSTR( "NO ack or Reserve Violated!\r\n" ));


    // This function shows how to transmit packets in a
    // non-blocking manner  
    // val = bmac_tx_pkt_nonblocking(tx_buf, strlen (tx_buf));
    // nrk_kprintf (PSTR ("Tx packet enqueued\r\n"));
    // This functions waits on the tx_done_signal
    // ret = nrk_event_wait (SIG(tx_done_signal));

    // Just check to be sure signal is okay
    // if(ret & SIG(tx_done_signal) == 0 ) 
    // nrk_kprintf (PSTR ("TX done signal error\r\n"));
   
    // If you want to see your remaining reservation
    // printf( "reserve=%d ",bmac_tx_reserve_get() );
    
    // Task gets control again after TX complete
    nrk_kprintf (PSTR ("Tx task sent data!\r\n"));
    nrk_led_clr (BLUE_LED);
    nrk_wait_until_next_period ();
  }

}
*/

// generate events
void event_task()
{
    uint8_t r;
    uint16_t cnt;
    uint32_t mac;
    nrk_time_t offset_time;

    // does tree-specific things
    test_setup();

    printf("evTask %d\r\n", nrk_get_pid());
    nrk_led_set(BLUE_LED);

    r=0;
    cnt = 0;
    begin = 0;
    end = 0;
    fill_cnt = 0;

    event_delay_time.secs = 20;
    event_delay_time.nano_secs = 0;

    bmac_data_send_sig = nrk_signal_create();

    read_eeprom_mac_address(&mac);
    my_addr8 = (mac & 0xFF);

    printf("addr %u\r\n", my_addr8);
    //bmac_init(18);
    while (!bmac_started())
        nrk_wait(delay_time);

    printf("bmac init done\r\n");

    // Auto ACK is an energy efficient link layer ACK on packets
    // If Auto ACK is enabled, then bmac_tx_pkt() will return failure
    // if no ACK was received. In a broadcast domain, the ACK's will
    // typically collide.  To avoid this, one can use address decoding. 
    // The functions are as follows:
    bmac_auto_ack_enable();
    // bmac_auto_ack_disable();

    // Address decoding is a way of preventing the radio from receiving
    // packets that are not address to a particular node.  This will 
    // supress ACK packets from nodes that should not automatically ACK.
    // The functions are as follows:
    bmac_addr_decode_set_my_mac(my_addr8); 
    //bmac_addr_decode_dest_mac(0xffff);  // 0xFFFF is broadcast
    bmac_addr_decode_dest_mac(my_parent);  // 0xFFFF is broadcast
    bmac_addr_decode_enable();
    // bmac_addr_decode_disable();

    //printf("NODE %d %d %d\r\n", tdma_mac_get(), tdma_tree_level_get(), tdma_tree_parent_get());

    // seed the number generator off mac address, then wait for 
    // random amount of time, to offset event generation
    srand(my_addr8);
    offset_time.secs = 0;
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

        //if ((tdma_my_mode_get()==TDMA_SLAVE) && (r==0))
        if (r == 0 && NODE_MODE == BMAC_SOURCE)
        {
            // only generate if I am a leaf node
            //if (tdma_tree_is_leaf())
            gen_event(my_addr8, cnt, my_level, 1);
            //add_event(my_addr8, cnt, my_level, 1);
            cnt = (cnt+1) % 65536;
            //  tdma_test_generate_event(cnt++);
            
           // sprintf( &tx_buf[BMAC_DATA_START], "Hello World %d", my_addr16 ); 
           // length=strlen(&tx_buf[BMAC_DATA_START])+BMAC_DATA_START;
           // tdma_tx_pkt( tx_buf, length);
            
        }
        //else
        //    nrk_kprintf(PSTR("No Event\r\n"));
        //nrk_led_clr(RED_LED);
        //PORTA &= ~BM(DEBUG_1);

        nrk_wait_until_next_period();
        //nrk_wait_until(event_delay_time);
        //event_delay_time.secs+=20;
            //printf("ITERATION %20"PRIu32"\r\n", event_delay_time.secs);

        //PORTA |= BM(DEBUG_1);
    }

}

void tx_task()
{ 

    uint8_t retries;

    printf("txTask %d\r\n", nrk_get_pid());
    have_data_to_send = 0;

    while(!bmac_started())
        nrk_wait(delay_time);

    tx_done_signal = bmac_get_tx_done_signal();
    nrk_signal_register(tx_done_signal);
    //srand(my_addr16);

    while(1)
    {

        while (!fill_cnt)
        {
            //nrk_wait(delay_time);
            //nrk_wait_until_next_period();
            nrk_signal_register(bmac_data_send_sig);
            nrk_event_wait(SIG(bmac_data_send_sig));

        }

        //if( bmac_tx_pkt_check()!=0 )
        //{
        //    nrk_kprintf(PSTR("Pending\r\n"));
            //printf( "Pending on slot %d\r\n",MY_TX_SLOT );
        //}
        //else
            nrk_led_set(RED_LED);
            //while (bmac_tx_pkt(test_tx_buf[current_buf], test_tx_offset[current_buf])==NRK_ERROR)
            //bmac_tx_pkt_nonblocking(test_tx_buf[current_buf], test_tx_offset[current_buf]);
            //    nrk_kprintf(PSTR("ackfail\r\n"));

            // switch buffers
            //current_buf = (current_buf + 1) % 2;
            //test_tx_offset[current_buf] = BMAC_DATA_START;
            //test_tx_buf[current_buf][BMAC_DATA_START] = 0;
            //have_data_to_send = 0;
            //nrk_led_clr(GREEN_LED);

            pkt_num++;
            fill_txbuf(&pkt_tx_buf, ev_queue[begin]);

            //while (bmac_tx_pkt(test_tx_buf[(current_buf+1)%2], test_tx_offset[(current_buf+1)%2])==NRK_ERROR);

            retries = 0;

            #ifdef BMAC_UNLIMITED_RETRY
                while (bmac_tx_pkt(pkt_tx_buf, EVPKT_SIZE)==NRK_ERROR);
            #else
                while (bmac_tx_pkt(pkt_tx_buf, EVPKT_SIZE)==NRK_ERROR && retries++ < MAX_RETRIES);
            #endif

        
        nrk_led_clr(RED_LED);

        //PORTA |= BM(DEBUG_1);
        //PORTA &= ~BM(DEBUG_1);

        // CHANGE 01/05/10 I will send the packet to my parent, the intended recipient, so that is the last field printed here
        nrk_time_get(&current_time);
        nrk_time_compact_nanos(&current_time);
        //printf("PKTSEND %u %u %u %u %20"PRIu32" %20"PRIu32"\r\n", my_addr8, pkt_num-1, my_level, tdma_tree_parent_get(), current_time.secs, current_time.nano_secs);
        //printf("PKTSEND %u %u %u %u %20"PRIu32" %20"PRIu32"\r\n", my_addr8, pkt_num-1, my_level, my_parent, current_time.secs, current_time.nano_secs);
        
#ifndef BMAC_UNLIMITED_RETRY
        if (retries < MAX_RETRIES)
            printf("PKTSEND %u %u %u %u\r\n", my_addr8, pkt_num-1, my_level, my_parent);
        else
            printf("GU %u %u\r\n", my_addr8, pkt_num-1);
#else
        // always sent
        printf("PKTSEND %u %u %u %u\r\n", my_addr8, pkt_num-1, my_level, my_parent);
#endif

    }
}

void rx_task()
{
    uint8_t *local_rx_buf;
    int8_t rssi;
    uint8_t length,slot;
    int8_t val;
    uint8_t i;
    uint8_t from_child;
    nrk_time_t check_period;

    
    printf("rxTask %d\r\n", nrk_get_pid());

    bmac_init(18);
    //while(!bmac_started())
    //    nrk_wait(delay_time);

    bmac_set_cca_thresh(-35); 
    bmac_rx_pkt_set_buffer(rx_buf, RF_MAX_PAYLOAD_SIZE);

    check_period.secs = 0;
    check_period.nano_secs = 100 * NANOS_PER_MS;
    bmac_set_rx_check_rate(check_period);


    while(1)
    {
        //printf("wait\r\n");
        val = bmac_wait_until_rx_pkt();
        nrk_led_set(ORANGE_LED);
        if (val == NRK_ERROR)
        {
            printf("Error waiting.\r\n");
            //nrk_terminate_task();
        }
        //printf("got\r\n");
        
        local_rx_buf=bmac_rx_pkt_get(&length, &rssi);
            //printf( "Got Packet len %d: ",length );

        from_child = 0;
        for (i = 0; i < num_children; i++)
        {
            if (local_rx_buf[BMAC_DATA_START+1] == child_addrs[i])
                from_child = 1;
        }

        if (from_child)
        {
            if (local_rx_buf[BMAC_DATA_START] == 0xF5)
            {
                if (NODE_MODE == BMAC_SOURCE)
                {
                    //tdma_test_route_event(local_rx_buf);
                    rte_event(local_rx_buf);
                }
                else
                {
                    //tdma_test_get_event(local_rx_buf);
                    get_event(local_rx_buf);
                }
            }
            else
                nrk_kprintf(PSTR("wrong CTL\r\n"));
        }
        else
            printf("Not from child %d\r\n", local_rx_buf[BMAC_DATA_START+2]);

        local_rx_buf[BMAC_DATA_START] = 1;
        bmac_rx_pkt_release();
        nrk_led_clr(ORANGE_LED);

        //PORTA |= BM(DEBUG_1);
        //PORTA &= ~BM(DEBUG_1);
    }
}

void stats_task()
{
    uint8_t r =0;

    printf("statTask %d\r\n", nrk_get_pid());
    //uint8_t mode = 0;

    //stats_delay_time.secs = 10;
    //stats_delay_time.nano_secs = 0;

    while(!bmac_started())
        nrk_wait(delay_time);

    while (1)
    {
        // print out stats every once in a while
        nrk_wait_until_next_period();

        // was mod 50
        r = (r+1) % 5;

        //nrk_wait_until(stats_delay_time);
        //stats_delay_time.secs+=10;

        if (r == 0)
            bmac_stats_dump();
        
    }

}


void nrk_create_taskset ()
{


  RX_TASK.task = rx_task;
  nrk_task_set_stk( &RX_TASK, rx_task_stack, STATS_STACK);
  RX_TASK.prio = 1;
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

  TX_TASK.task = tx_task;
  nrk_task_set_stk( &TX_TASK, tx_task_stack, STATS_STACK);
  TX_TASK.prio = 2;
  TX_TASK.FirstActivation = TRUE;
  TX_TASK.Type = BASIC_TASK;
  TX_TASK.SchType = PREEMPTIVE;
  TX_TASK.period.secs = 0;
  TX_TASK.period.nano_secs = 100000000;
  TX_TASK.cpu_reserve.secs = 0;
  TX_TASK.cpu_reserve.nano_secs = 0;
  TX_TASK.offset.secs = 0;
  TX_TASK.offset.nano_secs = 0;
  nrk_activate_task (&TX_TASK);

  evTask.task = event_task;
  //nrk_task_set_stk( &evTask, tx_task_stack, NRK_APP_STACKSIZE);
  evTask.Ptos = (void *) &ev_task_stack[STATS_STACK-1];
  evTask.Pbos = (void *) &ev_task_stack[0];
  evTask.prio = 2;
  evTask.FirstActivation = TRUE;
  evTask.Type = BASIC_TASK;
  evTask.SchType = PREEMPTIVE;
  evTask.period.secs = EVENT_PERIOD_SECS;
  evTask.period.nano_secs = EVENT_PERIOD_NANOS;
  evTask.cpu_reserve.secs = 0;
  evTask.cpu_reserve.nano_secs = 0;
  evTask.offset.secs = 0;
  evTask.offset.nano_secs = 0;
  nrk_activate_task (&evTask);

  statsTask.task = stats_task;
  //nrk_task_set_stk( &statsTask, tx_task_stack, NRK_APP_STACKSIZE);
  statsTask.Ptos = (void *) &stats_task_stack[STATS_STACK-1];
  statsTask.Pbos = (void *) &stats_task_stack[0];
  statsTask.prio = 1;
  statsTask.FirstActivation = TRUE;
  statsTask.Type = BASIC_TASK;
  statsTask.SchType = PREEMPTIVE;
  statsTask.period.secs = 20;
  statsTask.period.nano_secs = 0;
  statsTask.cpu_reserve.secs = 0;
  statsTask.cpu_reserve.nano_secs = 0;
  statsTask.offset.secs = 0;
  statsTask.offset.nano_secs = 0;
  nrk_activate_task (&statsTask);


  printf ("Create done\r\n");
}
