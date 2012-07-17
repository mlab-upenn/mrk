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

#define REPLY_WAIT_SECS	2

uint8_t subnet_mac[3];
uint8_t my_mac;
uint32_t mac_address;
//#define TXT_DEBUG

uint8_t aes_key[16] =
  { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b,
0x0c, 0x0d, 0x0e, 0x0f };

#define NLIST_SIZE	5
#define MAX_NEIGHBORS	16

typedef struct neighbor_list {
uint8_t mac[4];
int16_t rssi;
uint8_t rssi_cnt;
} LOCAL_N_LIST_T;


void build_extended_neighbor_list_pkt(SAMPL_PEER_2_PEER_PKT_T *p2p_pkt);
void build_ping_pkt(SAMPL_PEER_2_PEER_PKT_T *p2p_pkt);

nrk_task_type MOBILE_TASK;
NRK_STK mobile_task_stack[NRK_APP_STACKSIZE];
void tx_task (void);

void nrk_create_taskset ();


uint8_t tx_buf[RF_MAX_PAYLOAD_SIZE];
uint8_t rx_buf[RF_MAX_PAYLOAD_SIZE];
uint8_t cnt;

LOCAL_N_LIST_T my_nlist[MAX_NEIGHBORS];
uint8_t my_nlist_elements;

SAMPL_PEER_2_PEER_PKT_T p2p_pkt;

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

  bmac_task_config ();

  nrk_create_taskset ();
  nrk_start ();

  return 0;
}

void tx_task ()
{
  uint8_t j, i,  error,unique;
  uint8_t samples;
  int8_t len;
  int8_t rssi, val;
  int8_t tx_rssi;
  uint8_t *local_rx_buf;

  nrk_sig_t tx_done_signal;
  nrk_sig_t rx_signal;
  nrk_sig_mask_t ret;
  nrk_time_t check_period;
  nrk_time_t timeout, start, current;
  nrk_sig_mask_t my_sigs;

  printf ("tx_task PID=%d\r\n", nrk_get_pid ());

val = read_eeprom_mac_address (&mac_address);
  if (val == NRK_OK) {
    my_mac = (mac_address & 0xff);
    subnet_mac[0] = ((mac_address >> 8) & 0xff);
    subnet_mac[1] = ((mac_address >> 16) & 0xff);
    subnet_mac[2] = ((mac_address >> 24) & 0xff);
  }
  else {
    while (1) {
      nrk_led_toggle(RED_LED);
      nrk_kprintf (PSTR
                   ("* ERROR reading MAC address from EEPROM run eeprom-config utility\r\n"));
      //for(i=0; i<100; i++ ) nrk_wait_until_next_period ();
      nrk_wait_until_next_period ();
    }
  }
  val = read_eeprom_channel (&i);
  val = read_eeprom_aes_key ((uint8_t *) & aes_key);
  nrk_kprintf( PSTR("\r\nNano-RK Version: " ));
  printf( "%d",NRK_VERSION );
  nrk_kprintf( PSTR("\r\nSAMPL Version: " ));
  printf( "%d",SAMPL_VERSION );
  nrk_kprintf( PSTR("\r\nNetmask: 0x " ));
  printf ("%x", subnet_mac[2]);
  printf (" %x ", subnet_mac[1]);
  printf ("%x", subnet_mac[0]);
  nrk_kprintf( PSTR("\r\nMAC: 0x " ));
  printf ("%x", my_mac);
  nrk_kprintf( PSTR("\r\nChannel: " ));
  printf ("%d", i);



  // Wait until the tx_task starts up bmac
  // This should be called by all tasks using bmac that
  // do not call bmac_init()...
  bmac_init (i);

  nrk_kprintf( PSTR("\r\nAES Checksum: " ));
  j=0;
  for(i=0; i<32; i++ )
        {
        j+=aes_key[i];
        }
  printf ("%d", j);
  nrk_kprintf( PSTR("\r\n" ));

  bmac_encryption_set_key (aes_key, 16);
//  bmac_encryption_enable ();


  bmac_rx_pkt_set_buffer (rx_buf, RF_MAX_PAYLOAD_SIZE);

  val=bmac_addr_decode_set_my_mac(((uint16_t)subnet_mac[0]<<8)|my_mac);
  val=bmac_addr_decode_dest_mac(0xffff);  // broadcast by default
  bmac_addr_decode_enable();

  bmac_set_cca_thresh (-45);

  nrk_kprintf( PSTR( "Mobile MAC: " ));
  printf( "0x %x %x %x %x\r\n",subnet_mac[2], subnet_mac[1], subnet_mac[0], my_mac);

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

  while (1) {

  bmac_disable();
 do {
    nrk_wait_until_next_period ();
    } while(nrk_gpio_get(NRK_BUTTON)==1);

    bmac_enable();
    my_nlist_elements=0;

// clear average rssi data
for(i=0; i<MAX_NEIGHBORS; i++ )
	{
	my_nlist[i].rssi=0;	
	my_nlist[i].rssi_cnt=0;	
	}


for(samples=0; samples<3; samples++ )
{
    nrk_led_set (GREEN_LED);
    check_period.secs = 0;
    check_period.nano_secs = DEFAULT_CHECK_RATE * NANOS_PER_MS;
    val = bmac_set_rx_check_rate (check_period);
    build_ping_pkt( &p2p_pkt );
    // Pack data structure values in buffer before transmit
    pack_peer_2_peer_packet(&p2p_pkt);
    // For blocking transmits, use the following function call.
    val = bmac_tx_pkt (p2p_pkt.buf, p2p_pkt.buf_len);

    check_period.secs = 0;
    check_period.nano_secs = p2p_pkt.check_rate * NANOS_PER_MS;
    val = bmac_set_rx_check_rate (check_period);
#ifdef TXT_DEBUG
    nrk_kprintf (PSTR ("\r\nSent Ping Request:\r\n"));
#endif
    nrk_led_clr (GREEN_LED);

    // Wait for packets or timeout
    nrk_time_get (&start);
    while (1) {

      timeout.secs = REPLY_WAIT_SECS;
      timeout.nano_secs = 0;

      // Wait until an RX packet is received
      //val = bmac_wait_until_rx_pkt ();
      nrk_set_next_wakeup (timeout);
      my_sigs = nrk_event_wait (SIG (rx_signal) | SIG (nrk_wakeup_signal));


      if (my_sigs == 0)
        nrk_kprintf (PSTR ("Error calling nrk_event_wait()\r\n"));
      if (my_sigs & SIG (rx_signal)) {

        // Get the RX packet 
        local_rx_buf = bmac_rx_pkt_get (&len, &rssi);
	// Check the packet type from raw buffer before unpacking
        if ((local_rx_buf[CTRL_FLAGS] & (DS_MASK | US_MASK)) == 0) {

	// Set the buffer
	p2p_pkt.buf=local_rx_buf;
	p2p_pkt.buf_len=len;
	p2p_pkt.rssi=rssi;
	unpack_peer_2_peer_packet(&p2p_pkt);
	  // Check if newly received packet is for this node
          if (((p2p_pkt.dst_subnet_mac[2] == subnet_mac[2] &&
		p2p_pkt.dst_subnet_mac[1] == subnet_mac[1] &&
		p2p_pkt.dst_subnet_mac[0] == subnet_mac[0] &&
		p2p_pkt.dst_mac == my_mac) 
		|| p2p_pkt.dst_mac == BROADCAST) 
		    && p2p_pkt.pkt_type==RT_PING_PKT) {
		nrk_led_set(RED_LED);	
		// Packet arrived and is good to go
		tx_rssi=(int8_t)p2p_pkt.payload[1];
		#ifdef TXT_DEBUG
		nrk_kprintf( PSTR( "rx-rssi: "));
              	printf( "%d ",p2p_pkt.rssi);      
		nrk_kprintf( PSTR( "mac: "));
              	printf( "%u %u %u ",p2p_pkt.src_subnet_mac[0], p2p_pkt.src_subnet_mac[1], p2p_pkt.src_subnet_mac[2]);  
              	printf( "%u ",p2p_pkt.src_mac);  
              	
                nrk_kprintf (PSTR("RT-PING reply: "));
		nrk_kprintf( PSTR( "  mac: "));
                printf ("%u ", p2p_pkt.payload[0]);
		nrk_kprintf( PSTR( "  tx-rssi: "));
                printf ("%d\r\n", (int8_t)tx_rssi);
		#endif		
		unique=1;	
		// Check if the MAC is unique
		for(i=0; i<my_nlist_elements; i++ )
		{

		if(my_nlist[i].mac[3]==p2p_pkt.src_subnet_mac[2] &&
			my_nlist[i].mac[2]==p2p_pkt.src_subnet_mac[1] &&
			my_nlist[i].mac[1]==p2p_pkt.src_subnet_mac[0] &&
			my_nlist[i].mac[0]==p2p_pkt.src_mac)
			{
				my_nlist[i].rssi+=(int16_t)p2p_pkt.rssi;
				my_nlist[i].rssi_cnt++;
				my_nlist[i].rssi+=(int16_t)tx_rssi;
				my_nlist[i].rssi_cnt++;
				unique=0;
				break;
			}
		}

		// If MAC is unique, add it
		if(unique && my_nlist_elements<MAX_NEIGHBORS)
		{
			my_nlist[my_nlist_elements].mac[3]=p2p_pkt.src_subnet_mac[2];
			my_nlist[my_nlist_elements].mac[2]=p2p_pkt.src_subnet_mac[1];
			my_nlist[my_nlist_elements].mac[1]=p2p_pkt.src_subnet_mac[0];
			my_nlist[my_nlist_elements].mac[0]=p2p_pkt.src_mac;
			my_nlist[my_nlist_elements].rssi=(int16_t)p2p_pkt.rssi+(int16_t)tx_rssi;
			my_nlist[my_nlist_elements].rssi_cnt=2;
			my_nlist_elements++;
		}

		nrk_led_clr(RED_LED);	
            }

        }
        // Release the RX buffer so future packets can arrive 
        bmac_rx_pkt_release ();
      }

      nrk_time_get (&current);
      if (start.secs + REPLY_WAIT_SECS < current.secs)
        break;
    }
}
    check_period.secs = 0;
    check_period.nano_secs = DEFAULT_CHECK_RATE * NANOS_PER_MS;
    val = bmac_set_rx_check_rate (check_period);
   
    #ifdef TXT_DEBUG
       nrk_kprintf (PSTR ("Done Waiting for responses.\r\n"));
    #endif
    nrk_kprintf (PSTR ("\r\n\r\nSurvey Result:\r\n"));
   
    for(i=0; i<my_nlist_elements; i++ )
    {
	nrk_kprintf( PSTR( "MAC: " ));
	for(j=0; j<4; j++ )
	{
	if(my_nlist[i].mac[j]<0x10) printf( "0%x", my_nlist[i].mac[3-j] );
	else printf( "%x", my_nlist[i].mac[3-j] );
	}
	if(my_nlist[i].rssi==0)
		nrk_kprintf( PSTR(" RSSI: 0\r\n") );
	else
	{ 
          nrk_kprintf( PSTR( " RSSI: " ));
	  printf( "%d\r\n", (int8_t)((int16_t) my_nlist[i].rssi / (int16_t) my_nlist[i].rssi_cnt) );
        }
    }

    if(my_nlist_elements>0)
    {
    	nrk_led_set(BLUE_LED); 
    	nrk_kprintf (PSTR ("Uploading results...\r\n"));
    	build_extended_neighbor_list_pkt(&p2p_pkt);
    	// Pack data structure values in buffer before transmit
    	pack_peer_2_peer_packet(&p2p_pkt);
    	// For blocking transmits, use the following function call.
    	val = bmac_tx_pkt (p2p_pkt.buf, p2p_pkt.buf_len);
    	nrk_led_clr(BLUE_LED); 
    }
     
    
  }

}

void build_extended_neighbor_list_pkt(SAMPL_PEER_2_PEER_PKT_T *p2p_pkt)
{
uint8_t i;

    // Build a TX packet by hand...
    p2p_pkt->pkt_type = EXTENDED_NEIGHBOR_LIST_PKT;
    // set as p2p packet (no US_MASK or DS_MASK)
    p2p_pkt->ctrl_flags = MOBILE_MASK | LINK_ACK; // | DEBUG_FLAG ;  
    p2p_pkt->ack_retry= 0x0f;
    p2p_pkt->ttl = 5;
    // All flooding or routed packets must be default checkrate
    // only 1-hop reply messages can be less
    p2p_pkt->check_rate = DEFAULT_CHECK_RATE;
    p2p_pkt->src_subnet_mac[0] = subnet_mac[0];
    p2p_pkt->src_subnet_mac[1] = subnet_mac[1];
    p2p_pkt->src_subnet_mac[2] = subnet_mac[2];
    p2p_pkt->src_mac = my_mac;
    p2p_pkt->last_hop_mac = my_mac;
    p2p_pkt->dst_subnet_mac[0] = BROADCAST;
    p2p_pkt->dst_subnet_mac[1] = BROADCAST;
    p2p_pkt->dst_subnet_mac[2] = BROADCAST;
    p2p_pkt->dst_mac = 0;
    p2p_pkt->buf=tx_buf;
    p2p_pkt->buf_len = P2P_PAYLOAD_START;
    p2p_pkt->payload= &tx_buf[P2P_PAYLOAD_START];
    p2p_pkt->seq_num = cnt;
    cnt++;
    p2p_pkt->priority = 0;

    p2p_pkt->payload[0]=my_nlist_elements;
    for(i=0; i<my_nlist_elements; i++ )
	{
	p2p_pkt->payload[1+(i*NLIST_SIZE)]=my_nlist[i].mac[0];
	p2p_pkt->payload[1+(i*NLIST_SIZE)+1]=my_nlist[i].mac[1];
	p2p_pkt->payload[1+(i*NLIST_SIZE)+2]=my_nlist[i].mac[2];
	p2p_pkt->payload[1+(i*NLIST_SIZE)+3]=my_nlist[i].mac[3];
	if(my_nlist[i].rssi==0 ) 
		p2p_pkt->payload[1+(i*NLIST_SIZE)+4]=0;
	else
		p2p_pkt->payload[1+(i*NLIST_SIZE)+4]= (int8_t) (my_nlist[i].rssi / my_nlist[i].rssi_cnt);
	}

    p2p_pkt->payload_len=(my_nlist_elements*NLIST_SIZE)+1;

}

void build_ping_pkt(SAMPL_PEER_2_PEER_PKT_T *p2p_pkt)
{

    // Build a TX packet by hand...
    p2p_pkt->pkt_type = RT_PING_PKT;
    // set as p2p packet (no US_MASK or DS_MASK)
    p2p_pkt->ctrl_flags = MOBILE_MASK | LINK_ACK; // | DEBUG_FLAG ;  
    p2p_pkt->ack_retry= 0x0f;
    p2p_pkt->ttl = 1;
    p2p_pkt->check_rate = FAST_CHECK_RATE;
    p2p_pkt->src_subnet_mac[0] = subnet_mac[0];
    p2p_pkt->src_subnet_mac[1] = subnet_mac[1];
    p2p_pkt->src_subnet_mac[2] = subnet_mac[2];
    p2p_pkt->src_mac = my_mac;
    p2p_pkt->last_hop_mac = my_mac;
    p2p_pkt->dst_subnet_mac[0] = BROADCAST;
    p2p_pkt->dst_subnet_mac[1] = BROADCAST;
    p2p_pkt->dst_subnet_mac[2] = BROADCAST;
    p2p_pkt->dst_mac = BROADCAST;
    p2p_pkt->buf=tx_buf;
    p2p_pkt->buf_len = P2P_PAYLOAD_START;
    p2p_pkt->seq_num = cnt;
    p2p_pkt->priority = 0;
    p2p_pkt->payload= &tx_buf[P2P_PAYLOAD_START];
    // p2p_pkt->payload[0]=MY_MAC;
    p2p_pkt->payload_len=0;
    cnt++;
}

void nrk_create_taskset ()
{

  MOBILE_TASK.task = tx_task;
  nrk_task_set_stk (&MOBILE_TASK, mobile_task_stack, NRK_APP_STACKSIZE);
  MOBILE_TASK.prio = 2;
  MOBILE_TASK.FirstActivation = TRUE;
  MOBILE_TASK.Type = BASIC_TASK;
  MOBILE_TASK.SchType = PREEMPTIVE;
  MOBILE_TASK.period.secs = 1;
  MOBILE_TASK.period.nano_secs = 0;
  MOBILE_TASK.cpu_reserve.secs = 1;
  MOBILE_TASK.cpu_reserve.nano_secs = 500 * NANOS_PER_MS;
  MOBILE_TASK.offset.secs = 0;
  MOBILE_TASK.offset.nano_secs = 0;
  nrk_activate_task (&MOBILE_TASK);



}
