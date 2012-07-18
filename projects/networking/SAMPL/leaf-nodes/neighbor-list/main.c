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
#include <sampl.h>
#include <pkt_packer.h>
#include <nrk_driver_list.h>
#include <nrk_driver.h>
#include <nrk_timer.h>
#include <ff_basic_sensor.h>

#define REPLY_WAIT_SECS	1

// Full address = 0x01000064
#define MY_SUBNET_MAC_2		1
#define MY_SUBNET_MAC_1		0
#define MY_SUBNET_MAC_0		0
#define MY_MAC			100


#define TXT_DEBUG

#define NLIST_SIZE	5
#define MAX_NEIGHBORS	16

void build_extended_neighbor_list_pkt (SAMPL_PEER_2_PEER_PKT_T * p2p_pkt,
                                       uint8_t * nlist, uint8_t nlist_size);
void build_ping_pkt (SAMPL_PEER_2_PEER_PKT_T * p2p_pkt);
void nrk_register_drivers ();

nrk_task_type MOBILE_TASK;
NRK_STK mobile_task_stack[NRK_APP_STACKSIZE];
void tx_task (void);

void nrk_create_taskset ();


// tx_buf is used as the transmit buf for the neighborlist p2p packet
uint8_t tx_buf[RF_MAX_PAYLOAD_SIZE];
// rx_buf holds all incomming packets
uint8_t rx_buf[RF_MAX_PAYLOAD_SIZE];

uint8_t cnt;

uint8_t my_nlist[MAX_NEIGHBORS * NLIST_SIZE];
uint8_t my_nlist_elements;

SAMPL_PEER_2_PEER_PKT_T p2p_pkt;

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
  nrk_register_drivers ();

  bmac_task_config ();

  nrk_create_taskset ();
  nrk_start ();

  return 0;
}

void tx_task ()
{
  uint8_t i, unique;
  uint8_t samples ;
  uint8_t len;
  int8_t rssi, val;
  uint8_t *local_rx_buf;

  nrk_sig_t tx_done_signal;
  nrk_sig_t rx_signal;
  nrk_time_t check_period;
  nrk_time_t timeout, start, current;
  nrk_sig_mask_t my_sigs;

  printf ("tx_task PID=%d\r\n", nrk_get_pid ());


  // Wait until the tx_task starts up bmac
  // This should be called by all tasks using bmac that
  // do not call bmac_init()...
  bmac_init (26);

  bmac_rx_pkt_set_buffer (rx_buf, RF_MAX_PAYLOAD_SIZE);

  val =
    bmac_addr_decode_set_my_mac (((uint16_t) MY_SUBNET_MAC_0 << 8) | MY_MAC );
  val = bmac_addr_decode_dest_mac (0xffff);     // broadcast by default
  bmac_addr_decode_enable ();

  nrk_kprintf (PSTR ("bmac_started()\r\n"));
  bmac_set_cca_thresh (-45);


  check_period.secs = 0;
  check_period.nano_secs = 100 * NANOS_PER_MS;
  val = bmac_set_rx_check_rate (check_period);

  // Get and register the tx_done_signal if you want to
  // do non-blocking transmits
  tx_done_signal = bmac_get_tx_done_signal ();
  nrk_signal_register (tx_done_signal);

  rx_signal = bmac_get_rx_pkt_signal ();
  nrk_signal_register (rx_signal);

  cnt = 0;

  check_period.secs = 0;
  check_period.nano_secs = DEFAULT_CHECK_RATE * NANOS_PER_MS;
  val = bmac_set_rx_check_rate (check_period);


  // Main loop that does:
  //   1) Sends out ping message
  //   2) Collects replies, build neighbor list and then times out
  //   3) Repeat 1 and 2 for 3 times
  //   4) Build Extended Neighborlist packet
  //   5) Send Neighbor list packet
  //   6) Wait until next period and repeat 1-6
  while (1) {

    nrk_led_clr (ORANGE_LED);

    // Set our local neighbor list to be empty
    my_nlist_elements = 0;

    for (samples = 0; samples < 3; samples++) {
      nrk_led_set (GREEN_LED);
      check_period.secs = 0;
      check_period.nano_secs = DEFAULT_CHECK_RATE * NANOS_PER_MS;
      val = bmac_set_rx_check_rate (check_period);

      // Construct a ping packet to send (this is being built into tx_buf)
      build_ping_pkt (&p2p_pkt);

      // Pack data structure values in buffer before transmit
      pack_peer_2_peer_packet (&p2p_pkt);

      // Send the Ping packet 
      val = bmac_tx_pkt (p2p_pkt.buf, p2p_pkt.buf_len);

      // Set update rate based on p2p reply rate.
      // This is usually faster to limit congestion
      check_period.secs = 0;
      check_period.nano_secs = p2p_pkt.check_rate * NANOS_PER_MS;
      val = bmac_set_rx_check_rate (check_period);


#ifdef TXT_DEBUG
      nrk_kprintf (PSTR ("Pinging...\r\n"));
#endif
      nrk_led_clr (GREEN_LED);



      // Grab start time for timeout 
      nrk_time_get (&start);

      while (1) {

	// Set the amount of time to wait for timeout
        timeout.secs = REPLY_WAIT_SECS;
        timeout.nano_secs = 0;

	// Check if packet is already ready, or wait until one arrives
	// Also set timeout to break from function if no packets come
	my_sigs=0;
        if (bmac_rx_pkt_ready () == 0) {
          nrk_set_next_wakeup (timeout);
          my_sigs =
            nrk_event_wait (SIG (rx_signal) | SIG (nrk_wakeup_signal));
        }



        if (my_sigs == 0)
          nrk_kprintf (PSTR ("Error calling nrk_event_wait()\r\n"));
        if (my_sigs & SIG (rx_signal)) {

          // Get the RX packet 
          local_rx_buf = bmac_rx_pkt_get (&len, &rssi);
          // Check the packet type from raw buffer before unpacking
          if ((local_rx_buf[CTRL_FLAGS] & (DS_MASK | US_MASK)) == 0) {

            // Setup a p2p packet data structure with the newly received buffer 
            p2p_pkt.buf = local_rx_buf;
            p2p_pkt.buf_len = len;
            p2p_pkt.rssi = rssi;

            // Unpack the data from the array into the p2p_pkt data struct
            unpack_peer_2_peer_packet (&p2p_pkt);


	    // Check if newly received packet is for this node
          if (((p2p_pkt.dst_subnet_mac[2] == MY_SUBNET_MAC_2 &&
		p2p_pkt.dst_subnet_mac[1] == MY_SUBNET_MAC_1 &&
		p2p_pkt.dst_subnet_mac[0] == MY_SUBNET_MAC_0 &&
		p2p_pkt.dst_mac == MY_MAC ) 
		|| p2p_pkt.dst_mac == BROADCAST) 
                && (p2p_pkt.pkt_type == PING_PKT)) {
              // Packet arrived and is  ping pkt!
	      // Lets print some values out on the terminal
            printf ("full mac: %d %d %d %d ", p2p_pkt.src_subnet_mac[0],
                    p2p_pkt.src_subnet_mac[1], p2p_pkt.src_subnet_mac[2],
		    p2p_pkt.src_mac);
              printf ("rssi: %d ", p2p_pkt.rssi);
              printf ("type: %d ", p2p_pkt.pkt_type);
              nrk_kprintf (PSTR ("payload: ["));
              for (i = 0; i < p2p_pkt.payload_len; i++)
                printf ("%d ", p2p_pkt.payload[i]);
              nrk_kprintf (PSTR ("]\r\n"));

              unique = 1;
              // Check if the MAC of this ping is unique or if it already
              // exists in our neighbor list
              for (i = 0; i < my_nlist_elements; i++) {

                if (my_nlist[i * NLIST_SIZE] == p2p_pkt.src_subnet_mac[2] &&
                    my_nlist[i * NLIST_SIZE + 1] == p2p_pkt.src_subnet_mac[1] &&
                    my_nlist[i * NLIST_SIZE + 2] == p2p_pkt.src_subnet_mac[0] &&
                    my_nlist[i * NLIST_SIZE + 3] == p2p_pkt.src_mac) {
                  unique = 0;
                  break;
                }
              }

              // If MAC is unique, add it to our neighbor list
              if (unique) {
                my_nlist[my_nlist_elements * NLIST_SIZE] =
                  p2p_pkt.src_subnet_mac[2];
                my_nlist[my_nlist_elements * NLIST_SIZE + 1] =
                  p2p_pkt.src_subnet_mac[1];
                my_nlist[my_nlist_elements * NLIST_SIZE + 2] =
                  p2p_pkt.src_subnet_mac[0];
                my_nlist[my_nlist_elements * NLIST_SIZE + 3] =
                  p2p_pkt.src_mac;
                my_nlist[my_nlist_elements * NLIST_SIZE + 4] = p2p_pkt.rssi;
                my_nlist_elements++;
              }
            }

          }


        }

	// Check if we are done waiting for pings
        nrk_time_get (&current);
        if (start.secs + REPLY_WAIT_SECS < current.secs)
          break; // exit loops waiting for pings
	
          // Release the RX buffer so future packets can arrive 
          bmac_rx_pkt_release ();

      // Go back to top loop to wait for more pings
      }
      cnt++;
    // Repeat ping 3 times
    }

    // Now we are ready to build extended neighborlist packet and send it to gateway
    check_period.secs = 0;
    check_period.nano_secs = DEFAULT_CHECK_RATE * NANOS_PER_MS;
    val = bmac_set_rx_check_rate (check_period);

    nrk_kprintf (PSTR ("Done Waiting for response...\r\n"));

    // If we have any neighbors, build the list
    if (my_nlist_elements > 0) {
      // Look in this function for format of extended neighborlist packet
      // This function also configures the parameters and destination address
      // of the p2p packet.  The values are probably okay as defaults.
      build_extended_neighbor_list_pkt (&p2p_pkt, my_nlist,
                                        my_nlist_elements);
      // This function takes at p2p struct and packs it into an array for sending
      pack_peer_2_peer_packet (&p2p_pkt);

      nrk_led_set (BLUE_LED);
      // Send the list to the gateway.
      val = bmac_tx_pkt (p2p_pkt.buf, p2p_pkt.buf_len);
      printf ("size of pkt: %d\r\n", p2p_pkt.buf_len);
      nrk_kprintf (PSTR ("sent neighbor list packet\r\n"));
      nrk_led_clr (BLUE_LED);
    }
    else {
      nrk_led_set (RED_LED);
      nrk_spin_wait_us (1000);
      nrk_led_clr (RED_LED);
    }

    // Wait a long time until we send out the pings again
    // This is in a loop so that period can be small for 
    // other uses.
    for (i = 0; i < 10; i++)
      nrk_wait_until_next_period ();

    // Might as well release packets that arrived during long
    // break since they are not replies to your ping.
    bmac_rx_pkt_release ();

  }

}



// This function takes a p2p data structure and a list of extended
// neighbors and builds a proper Extended Neighbor list packet for the
// gateway.  The extended neighbor list contains the full address of all
// neighbors so that it can work between subnets.
void build_extended_neighbor_list_pkt (SAMPL_PEER_2_PEER_PKT_T * p2p_pkt,
                                       uint8_t *nlist, uint8_t nlist_size)
{
  uint8_t i;

  // Set SAMPL packet type to extended neighbor list...
  p2p_pkt->pkt_type = EXTENDED_NEIGHBOR_LIST_PKT;
  p2p_pkt->ctrl_flags = MOBILE_MASK;    // | DEBUG_FLAG ;  
  p2p_pkt->ack_retry = 0x00;
  p2p_pkt->ttl = 5;
  // All flooding or routed packets must be default checkrate
  // only 1-hop reply messages can be less
  p2p_pkt->check_rate = DEFAULT_CHECK_RATE;
  p2p_pkt->src_subnet_mac[0] = MY_SUBNET_MAC_0;
  p2p_pkt->src_subnet_mac[1] = MY_SUBNET_MAC_1;
  p2p_pkt->src_subnet_mac[2] = MY_SUBNET_MAC_2;
  p2p_pkt->src_mac = MY_MAC;
  p2p_pkt->last_hop_mac = MY_MAC;
  // Destination 0 is the gateway
  p2p_pkt->dst_subnet_mac[0] = BROADCAST;
  p2p_pkt->dst_subnet_mac[1] = BROADCAST;
  p2p_pkt->dst_subnet_mac[2] = BROADCAST;
  p2p_pkt->dst_mac = 0;
  p2p_pkt->buf = tx_buf;
  // Configre packet with no payload to start
  // We will update buffer length later
  p2p_pkt->buf_len = P2P_PAYLOAD_START;
  // Connect p2p payload packet with tx_buf
  p2p_pkt->payload = &tx_buf[P2P_PAYLOAD_START];
  // Sequence number doesn't really matter for p2p packets
  p2p_pkt->seq_num = cnt;
  p2p_pkt->priority = 0;

  // Add Extended neighbor list payload
  // Byte 0 is the number of neighbor elements
  // Each neighbor element is 5 bytes
  //    byte 0 - uint8_t MAC addr byte 3 
  //    byte 1 - uint8_t MAC addr byte 2 
  //    byte 2 - uint8_t MAC addr byte 1 
  //    byte 3 - uint8_t MAC addr byte 0 
  //    byte 4 - int8_t RSSI value 
  // These are all neighbors of the p2p_pkt source address
  // which can be found in the header.
  //
  //  Ex: 0x2 0x0 0x0 0x0 0x3 0xff 0x0 0x1 0x0 0x2 0xff
  //      2 neighbors: 0x00000003 with rssi -1
  //                   0x00010002 with rssi -1
 
  // Add number of elements 
  p2p_pkt->payload[0] = nlist_size;

  // Pack neighbor info
  for (i = 0; i < nlist_size * NLIST_SIZE; i++)
    p2p_pkt->payload[i + 1] = nlist[i];

  // Update the total payload size (+1 for the number of elements byte)
  p2p_pkt->payload_len = nlist_size * NLIST_SIZE + 1;

}


// This function builds a 1-hop p2p ping packet
void build_ping_pkt (SAMPL_PEER_2_PEER_PKT_T * p2p_pkt)
{

  p2p_pkt->pkt_type = PING_PKT;
  p2p_pkt->ctrl_flags = MOBILE_MASK;    // | DEBUG_FLAG ;  
  p2p_pkt->ack_retry = 0x00;
  p2p_pkt->ttl = 1;
  p2p_pkt->check_rate = 50;
  p2p_pkt->src_subnet_mac[0] = MY_SUBNET_MAC_0;
  p2p_pkt->src_subnet_mac[1] = MY_SUBNET_MAC_1;
  p2p_pkt->src_subnet_mac[2] = MY_SUBNET_MAC_2;
  p2p_pkt->src_mac = MY_MAC;
  p2p_pkt->last_hop_mac = MY_MAC;
  p2p_pkt->dst_subnet_mac[0] = BROADCAST;
  p2p_pkt->dst_subnet_mac[1] = BROADCAST;
  p2p_pkt->dst_subnet_mac[2] = BROADCAST;
  p2p_pkt->dst_mac = BROADCAST;
  p2p_pkt->buf = tx_buf;
  p2p_pkt->buf_len = P2P_PAYLOAD_START;
  p2p_pkt->seq_num = cnt;
  p2p_pkt->priority = 0;
  p2p_pkt->payload = &tx_buf[P2P_PAYLOAD_START];
  p2p_pkt->payload[0] = MY_MAC;
  p2p_pkt->payload_len = 1;
}

void nrk_create_taskset ()
{
// Create the task that does the work for us

  MOBILE_TASK.task = tx_task;
  nrk_task_set_stk (&MOBILE_TASK, mobile_task_stack, NRK_APP_STACKSIZE);
  MOBILE_TASK.prio = 2;
  MOBILE_TASK.FirstActivation = TRUE;
  MOBILE_TASK.Type = BASIC_TASK;
  MOBILE_TASK.SchType = PREEMPTIVE;
  MOBILE_TASK.period.secs = 0;
  MOBILE_TASK.period.nano_secs = 500 * NANOS_PER_MS;
  MOBILE_TASK.cpu_reserve.secs = 1;
  MOBILE_TASK.cpu_reserve.nano_secs = 500 * NANOS_PER_MS;
  MOBILE_TASK.offset.secs = 0;
  MOBILE_TASK.offset.nano_secs = 0;
  nrk_activate_task (&MOBILE_TASK);



}

void nrk_register_drivers ()
{
  int8_t val;

// Register the Basic FireFly Sensor device driver
// Make sure to add: 
//     #define NRK_MAX_DRIVER_CNT  
//     in nrk_cfg.h
// Make sure to add: 
//     SRC += $(ROOT_DIR)/src/drivers/platform/$(PLATFORM_TYPE)/source/ff_basic_sensor.c
//     in makefile

// Might use this later for reading accelerometer etc
  val = nrk_register_driver (&dev_manager_ff_sensors, FIREFLY_SENSOR_BASIC);
  if (val == NRK_ERROR)
    nrk_kprintf (PSTR ("Failed to load my ADC driver\r\n"));

}
