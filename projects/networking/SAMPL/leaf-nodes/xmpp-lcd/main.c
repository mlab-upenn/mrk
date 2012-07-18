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
#include <globals.h>
#include <sampl.h>
#include <pkt_packer.h>
#include <xmpp_pkt.h>
#include <ff_lcd.h>
#include <lcd_driver.h>

#define REPLY_WAIT_SECS	3

// Full address = 0x00000064
#define MY_SUBNET_MAC_2		0
#define MY_SUBNET_MAC_1		0
#define MY_SUBNET_MAC_0		0
#define MY_MAC			100


//#define TXT_DEBUG

char my_passwd[32];
char dst_jid[64];
char src_jid[64];
char tst_msg[64];

nrk_task_type TX_TASK;
NRK_STK tx_task_stack[NRK_APP_STACKSIZE];
void tx_task (void);

nrk_task_type RX_TASK;
NRK_STK rx_task_stack[NRK_APP_STACKSIZE];
void rx_task (void);

nrk_task_type CYCLING_TASK;
NRK_STK cycling_task_stack[NRK_APP_STACKSIZE];
void cycling_task(void);


XMPP_PKT_T xp, xp_in;

void nrk_create_taskset ();


uint8_t tx_buf[RF_MAX_PAYLOAD_SIZE];
uint8_t rx_buf[RF_MAX_PAYLOAD_SIZE];

nrk_time_t check_period;
nrk_time_t timeout, start, current;


SAMPL_PEER_2_PEER_PKT_T p2p_pkt;

int main ()
{
  nrk_setup_ports();
  nrk_setup_uart(UART_BAUDRATE_115K2);

  nrk_init();

  nrk_led_clr(0);
  nrk_led_clr(1);
  nrk_led_clr(2);
  nrk_led_clr(3);

  nrk_time_set(0, 0);

  bmac_task_config();

  nrk_create_taskset();
  nrk_start();

  return 0;
}

// This task periodically sends and XMPP message through the gateway
// to an arbitrary JID on the network.
void tx_task ()
{
  uint8_t cnt ;
  int8_t val;


  printf ("tx_task PID=%d\r\n", nrk_get_pid ());

  // Configure address for other packet handlers (my not be needed)
  my_mac= MY_MAC;
  my_subnet_mac[0]= MY_SUBNET_MAC_0;
  my_subnet_mac[1]= MY_SUBNET_MAC_1;
  my_subnet_mac[2]= MY_SUBNET_MAC_2;
  mac_address= (uint8_t)MY_SUBNET_MAC_2 << 24 | (uint8_t)MY_SUBNET_MAC_1 <<16 | (uint8_t)MY_SUBNET_MAC_0 << 8 | (uint8_t)MY_MAC; 



  // Wait until the tx_task starts up bmac
  // This should be called by all tasks using bmac that
  // do not call bmac_init()...
  bmac_init (15); //channel 15

  bmac_rx_pkt_set_buffer (rx_buf, RF_MAX_PAYLOAD_SIZE);

  val =
    bmac_addr_decode_set_my_mac (((uint16_t) MY_SUBNET_MAC_0 << 8) | MY_MAC);
  val = bmac_addr_decode_dest_mac (0xffff);     // broadcast by default
  bmac_addr_decode_enable ();

  nrk_kprintf (PSTR ("bmac_started()\r\n"));
  bmac_set_cca_thresh (-45);




  cnt = 0;

  while (1) {


    // Build an XMPP p2p packet
    p2p_pkt.pkt_type = XMPP_PKT;
    p2p_pkt.ctrl_flags = LINK_ACK | MOBILE_MASK;
    p2p_pkt.ack_retry = 0xf0;
    p2p_pkt.ttl = 5;
    p2p_pkt.src_subnet_mac[0] = MY_SUBNET_MAC_0;
    p2p_pkt.src_subnet_mac[1] = MY_SUBNET_MAC_1;
    p2p_pkt.src_subnet_mac[2] = MY_SUBNET_MAC_2;
    p2p_pkt.src_mac = MY_MAC;
    p2p_pkt.last_hop_mac = MY_MAC;
    // Set destination to be any nearby gateway
    p2p_pkt.dst_subnet_mac[0] = BROADCAST;
    p2p_pkt.dst_subnet_mac[1] = BROADCAST;
    p2p_pkt.dst_subnet_mac[2] = BROADCAST;
    p2p_pkt.dst_mac = 0;
    p2p_pkt.buf = tx_buf;
    p2p_pkt.buf_len = P2P_PAYLOAD_START;
    p2p_pkt.seq_num = cnt;
    p2p_pkt.priority = 0;
    p2p_pkt.check_rate = 100;
    p2p_pkt.payload = &(tx_buf[P2P_PAYLOAD_START]);


    // Create XMPP lite message that includes:
    //   1) destination JID
    //   2) Your node's password
    //   3) timeout value in seconds
    //   4) Message as a string (only ascii text, no XML delimiters)
    // 
    // The total size of the packet must be under 110 bytes
  
    // CHANGE THIS FIRST!
    // Setting binary_flag to 1 will convert message into ASCII HEX format
    // Setting binary_flag to 0 will send ASCII string 
    xp.binary_flag=0;
    xp.pub_sub_flag=0;
    xp.explicit_src_jid_flag=0;
    xp.src_jid_size=0;
    sprintf (my_passwd, "firefly");
    xp.passwd = my_passwd;  // This is the mobile node JID's password
    sprintf (dst_jid, "vrajkuma3");
    //sprintf (dst_jid, "agr@sensor-test.andrew.cmu.edu");
    // JIDs without the @ symbol are set to the gateway's server
    xp.dst_jid = dst_jid;  // This is the destination of the message
    sprintf (tst_msg, "omg, I'm trapped in an elevator, lol %d", cnt);
    xp.msg = tst_msg;  // This is the message body
    xp.timeout = 30;   // 30 second timeout for the connection
                       // After 30 seconds, the gateway will log the node out
   
     
    // If you need to login as a different user
     xp.explicit_src_jid_flag=1;
     sprintf (src_jid, "vrajkuma2");
     xp.src_jid = src_jid; 
     xp.src_jid_size=strlen(src_jid)+1;
   


    xp.dst_jid_size=strlen(dst_jid)+1;
    xp.passwd_size=strlen(my_passwd)+1;
    xp.msg_size=strlen(tst_msg)+1;
    // Pack the XMPP data structure into a byte sequence for sending
    p2p_pkt.payload_len = xmpp_pkt_pack (&xp, p2p_pkt.payload, 0);

    // Make sure it isn't too long!
    if (p2p_pkt.payload_len == 0) {
      nrk_kprintf (PSTR ("XMPP packet too long!\r\n"));
      nrk_wait_until_next_period ();
      continue;
    }
    cnt++;

    // Lets print out what we are sending
    printf ("Msg to: %s\r\n", xp.src_jid);
    printf ("  body: %s\r\n", xp.msg);

    nrk_led_set (BLUE_LED);

    // Make sure bmac checkrate is correct
    check_period.secs = 0;
    check_period.nano_secs = DEFAULT_CHECK_RATE * NANOS_PER_MS;
    val = bmac_set_rx_check_rate (check_period);

    // Pack data structure values in buffer before transmit
    pack_peer_2_peer_packet (&p2p_pkt);
    // For blocking transmits, use the following function call.
    val = bmac_tx_pkt (p2p_pkt.buf, p2p_pkt.buf_len);


    nrk_led_clr (BLUE_LED);
    nrk_led_clr (GREEN_LED);

    nrk_wait_until_next_period ();
  }

}


// This task listens for messages and if it receives an XMPP
// message addressed to broadcast or this node, it will print it out
void rx_task ()
{
  uint8_t len;
  int8_t rssi, val;
  uint8_t *local_rx_buf;
  XMPP_PKT_T rxp;
  nrk_time_t check_period;
  printf ("rx_task PID=%d\r\n", nrk_get_pid ());

  // Initialize FireFly LCD v1.2 board
  lcd_setup();
  lcd_string_display_escape("Waiting\\nfor msg.");

  while (!bmac_started ())
    nrk_wait_until_next_period ();

  check_period.secs = 0;
  check_period.nano_secs = 100 * NANOS_PER_MS;
  val = bmac_set_rx_check_rate (check_period);

  while (1) {

    // Wait for new packet if one isn't already available
    if (bmac_rx_pkt_ready () == 0) {
      val = bmac_wait_until_rx_pkt ();
    }

    // Get the RX packet 
    local_rx_buf = bmac_rx_pkt_get (&len, &rssi);
    // Check the packet type from raw buffer before unpacking
    if ((local_rx_buf[CTRL_FLAGS] & (MOBILE_MASK)) != 0) {

      // Set the buffer
      p2p_pkt.buf = local_rx_buf;
      p2p_pkt.buf_len = len;
      p2p_pkt.rssi = rssi;
      unpack_peer_2_peer_packet (&p2p_pkt);
      // just check the last part of the address because we are lazy
      // In the future you should check the whole address so that you
      // don't accidentally get someone elses messages

          if ((p2p_pkt.dst_subnet_mac[2] == MY_SUBNET_MAC_2 &&
		p2p_pkt.dst_subnet_mac[1] == MY_SUBNET_MAC_1 &&
		p2p_pkt.dst_subnet_mac[0] == MY_SUBNET_MAC_0 &&
		p2p_pkt.dst_mac == MY_MAC ) 
		|| p2p_pkt.dst_mac == BROADCAST) 
	{
        nrk_led_set (GREEN_LED);
        // Packet arrived and is good to go
#ifdef TXT_DEBUG
        printf ("full mac: %d %d %d %d ", p2p_pkt.src_subnet_mac[0],
                    p2p_pkt.src_subnet_mac[1], p2p_pkt.src_subnet_mac[2],
		    p2p_pkt.src_mac);
        printf ("rssi: %d ", p2p_pkt.rssi);
        printf ("type: %d ", p2p_pkt.pkt_type);
        nrk_kprintf (PSTR ("payload: ["));
        for (i = p2p_pkt.payload_start; i < p2p_pkt.buf_len; i++)
          printf ("%d ", p2p_pkt.buf[i]);
        nrk_kprintf (PSTR ("]\r\n"));
#endif
        // If it is an XMPP message
        if (p2p_pkt.pkt_type == XMPP_PKT) {
          nrk_kprintf (PSTR ("XMPP packet for me!\r\n"));

          // Unpack the message
          xmpp_pkt_unpack (&rxp, p2p_pkt.payload, 0);

          // Print it out
          printf ("XMPP msg:\r\n");
          printf ("  seq-num=%d\r\n", p2p_pkt.seq_num);
          printf ("  from-jid=%s\r\n", rxp.src_jid);
          printf ("  msg=%s\r\n", rxp.msg);

          // Load and display message
          lcd_string_array_load(rxp.msg);
        }

      }
    }
    // Release the RX buffer so future packets can arrive 
    bmac_rx_pkt_release ();

  }
}

void cycling_task()
{
  while(1)
  {
    if(lcd_string_array_autocycling_get())
    {
      lcd_string_array_cycle();
      lcd_wait_us(LCD_AUTOCYCLING_TIME);
    }
    else
    {
      if(lcd_switch_pressed(1))
        lcd_string_array_cycle();
      else if(lcd_switch_pressed(2))
        lcd_string_array_cycle_reverse();

      nrk_wait_until_next_period();
    }
  }
}

void nrk_create_taskset ()
{
  TX_TASK.task = tx_task;
  nrk_task_set_stk (&TX_TASK, tx_task_stack, NRK_APP_STACKSIZE);
  TX_TASK.prio = 2;
  TX_TASK.FirstActivation = TRUE;
  TX_TASK.Type = BASIC_TASK;
  TX_TASK.SchType = PREEMPTIVE;
  TX_TASK.period.secs = 10;
  TX_TASK.period.nano_secs = 0;
  TX_TASK.cpu_reserve.secs = 1;
  TX_TASK.cpu_reserve.nano_secs = 500 * NANOS_PER_MS;
  TX_TASK.offset.secs = 0;
  TX_TASK.offset.nano_secs = 0;
  nrk_activate_task (&TX_TASK);

  RX_TASK.task = rx_task;
  nrk_task_set_stk (&RX_TASK, rx_task_stack, NRK_APP_STACKSIZE);
  RX_TASK.prio = 3;
  RX_TASK.FirstActivation = TRUE;
  RX_TASK.Type = BASIC_TASK;
  RX_TASK.SchType = PREEMPTIVE;
  RX_TASK.period.secs = 10;
  RX_TASK.period.nano_secs = 0;
  RX_TASK.cpu_reserve.secs = 1;
  RX_TASK.cpu_reserve.nano_secs = 500 * NANOS_PER_MS;
  RX_TASK.offset.secs = 0;
  RX_TASK.offset.nano_secs = 0;
  nrk_activate_task (&RX_TASK);

  CYCLING_TASK.task = cycling_task;
  nrk_task_set_stk(&CYCLING_TASK, cycling_task_stack, NRK_APP_STACKSIZE);
  CYCLING_TASK.prio = 1;
  CYCLING_TASK.FirstActivation = TRUE;
  CYCLING_TASK.Type = BASIC_TASK;
  CYCLING_TASK.SchType = PREEMPTIVE;
  CYCLING_TASK.period.secs = 0;
  CYCLING_TASK.period.nano_secs = 100 * NANOS_PER_MS;
  CYCLING_TASK.cpu_reserve.secs = 1;
  CYCLING_TASK.cpu_reserve.nano_secs = 50 * NANOS_PER_MS;
  CYCLING_TASK.offset.secs = 0;
  CYCLING_TASK.offset.nano_secs = 0;
  nrk_activate_task(&CYCLING_TASK);
}
