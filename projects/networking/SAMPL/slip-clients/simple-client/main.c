#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <sampl.h>
#include <slipstream.h>
#include <ack_pkt.h>
#include <ff_basic_sensor_pkt.h>
#include <transducer_pkt.h>
#include <transducer_registry.h>
#include <pkt_debug.h>


uint32_t gw_mac_addr_full;
uint8_t gw_subnet_2;
uint8_t gw_subnet_1;
uint8_t gw_subnet_0;
uint8_t gw_mac;
uint8_t debug_txt_flag;

#define NONBLOCKING  0
#define BLOCKING     1

#define HEX_STR_SIZE	5

void error (char *msg);
void print_ds_packet (SAMPL_DOWNSTREAM_PKT_T * ds_pkt);
void print_gw_packet (SAMPL_GATEWAY_PKT_T * gw_pkt);

int sockfd, portno, n;
struct sockaddr_in serv_addr;
struct hostent *server;
int size;
char buffer[2048];

uint8_t my_buf[32];
uint8_t tran_pkt_buf[128];

SAMPL_DOWNSTREAM_PKT_T ds_pkt;

int main (int argc, char *argv[])
{
  FILE *fp;
  uint8_t tx_buf[128];
  uint8_t rx_buf[128];
  TRANSDUCER_PKT_T tran_pkt;
  TRANSDUCER_MSG_T tran_msg;
  SAMPL_GATEWAY_PKT_T gw_pkt;
  int32_t v, cnt, i, len ;
  uint8_t nav_time_secs, reply_time_secs, checksum;
  int32_t tmp;
  time_t reply_timeout, nav_timeout, t;
  uint8_t cmd, error;
  char buf[1024];
  time_t clockt;
  uint32_t epocht;

  debug_txt_flag = 0;

  if (argc < 3 || argc > 4) {
    printf ("Usage: server port mac-addr [-d]\n");
    printf ("  d Debug Output\n\n");
    printf ("example usage: ./simple-client localhost 5000 0x00000000\n");
    exit (1);
  }

  if (argc == 5) {
    // Grab dash command line options
    if (strstr (argv[4], "d") != NULL) {
      debug_txt_flag = 1;
    }
  }


  v = slipstream_open (argv[1], atoi (argv[2]), NONBLOCKING);

 if (strlen (argv[3]) != 8 && strlen (argv[3]) != 10) {
    printf ("Invalid MAC address\n");
  }
  v = sscanf (argv[3], "%x", &gw_mac_addr_full);
  if (v != 1) {
    printf ("Invalid MAC address\n");
  }
  printf ("GW mac: 0x%08x\n", gw_mac_addr_full);
  gw_subnet_2 = gw_mac_addr_full >> 24;
  gw_subnet_1 = gw_mac_addr_full >> 16;
  gw_subnet_0 = gw_mac_addr_full >> 8;
  gw_mac = gw_mac_addr_full & 0xff;


  nav_time_secs = 25;

  cnt = 0;
  while (1) {
    error = 0;
    cmd = 0;


  retry:
    // Setup the packet to send out to the network

    // These values setup the internal data structure and probably don't
    // need to be changed
    ds_pkt.payload_len = 0;
    ds_pkt.buf = tx_buf;
    ds_pkt.buf_len = DS_PAYLOAD_START;
    ds_pkt.payload_start = DS_PAYLOAD_START;
    ds_pkt.payload = &(tx_buf[DS_PAYLOAD_START]);

    // These are parameters that can be adjusted for different packets
    //  ds_pkt.pkt_type=PING_PKT; 
    ds_pkt.pkt_type = TRANSDUCER_PKT;
    ds_pkt.ctrl_flags =
      DS_MASK  | LINK_ACK | DEBUG_FLAG | ENCRYPT /*| TREE_FILTER*/  ;
    ds_pkt.seq_num = 0;
    ds_pkt.priority = 0;
    ds_pkt.ack_retry = 0x5;
    ds_pkt.subnet_mac[0] = gw_subnet_0;
    ds_pkt.subnet_mac[1] = gw_subnet_1;
    ds_pkt.subnet_mac[2] = gw_subnet_2;
    ds_pkt.hop_cnt = 0;         // Starting depth, always keep at 0
    ds_pkt.hop_max = 6;         // Max tree depth
    ds_pkt.delay_per_level = 2; // Reply delay per level in seconds
    ds_pkt.nav = 30;            // Time in seconds until next message to be sent
    ds_pkt.mac_check_rate = 100;        // B-mac check rate in ms
    ds_pkt.rssi_threshold = -40;        // Reply RSSI threshold
    ds_pkt.last_hop_mac = 0;
    ds_pkt.mac_filter_num = 0;  // Increase if MAC_FILTER is active
    clockt = time(NULL);
    epocht = clockt;
    ds_pkt.epoch_time[0] = epocht & 0xff;      // Epoch time 
    ds_pkt.epoch_time[1] = (epocht>>8) & 0xff;    
    ds_pkt.epoch_time[2] = (epocht>>16) & 0xff;    
    ds_pkt.epoch_time[3] = (epocht>>24) & 0xff;    
    //     ds_pkt.payload_len=0; // for ping pkt

    //  Add MAC filter entries below
    //  Do this before application packets are added.
    // ds_pkt.mac_filter_num = 1;  // Increase if MAC_FILTER is active
    //  downstream_packet_add_mac_filter( &ds_pkt, 0x3 ); 
    //  downstream_packet_add_mac_filter( &ds_pkt, 13 ); 
    //  downstream_packet_add_mac_filter( &ds_pkt, 4 ); 
    //  downstream_packet_add_mac_filter( &ds_pkt, 5 ); 

    tran_pkt.num_msgs = 0;
    tran_pkt.checksum = 0;
    tran_pkt.msgs_payload = tran_pkt_buf; 

    // Request sensor values
    tran_msg.mac_addr = 0xff;
    tran_msg.type=TRAN_FF_BASIC_SHORT;
    tran_msg.len= 0;
    tran_msg.payload = my_buf;
   
    transducer_msg_add( &tran_pkt, &tran_msg );

/*
    // Example of blinking the RED and BLUE leds
    tran_msg.mac_addr = 0xff;
    tran_msg.type=TRAN_LED_BLINK;
    tran_msg.len= 1;
       my_buf[0]=TRAN_RED_LED_MASK | TRAN_BLUE_LED_MASK;  	
    tran_msg.payload = my_buf;
   
    transducer_msg_add( &tran_pkt, &tran_msg );
*/



    ds_pkt.payload_len =
      transducer_pkt_pack(&tran_pkt,ds_pkt.payload);
    printf( "payload len= %d\n",ds_pkt.payload_len );

    // This takes the structure and packs it into the raw
    // array that is sent using SLIP
    pack_downstream_packet (&ds_pkt);

    printf ("Raw CMD str:\n");
    for (i = 0; i < ds_pkt.buf_len; i++)
      printf ("0x%02x ", ds_pkt.buf[i]);
    printf ("\n");
    // Print your packet on the screen
    if (debug_txt_flag == 1)
      print_ds_packet (&ds_pkt);

    printf ("Raw: ");
    for (i = 0; i < ds_pkt.buf_len; i++)
      printf ("0x%02x ", ds_pkt.buf[i]);
    printf ("\n");
    //cnt++;

    if (error == 0)
    	v=slipstream_acked_send(ds_pkt.buf, ds_pkt.buf_len, 3 ); //  v = slipstream_send (ds_pkt.buf, ds_pkt.buf_len);



    if (v == 0) {
      printf ("Error, no serial reply from gateway!\n");
      goto retry;
    }


    if (debug_txt_flag == 1) {
      if (v == 0)
        printf ("Error sending\n");
      else
        printf ("Sent request %d\n", ds_pkt.seq_num);
    }

    nav_time_secs = ds_pkt.nav;
    reply_time_secs = ds_pkt.delay_per_level * ds_pkt.hop_max;

    t = time (NULL);
    reply_timeout = t + reply_time_secs + 1;
    nav_timeout = t + nav_time_secs;

    // Collect Reply packets 
    while (reply_timeout > time (NULL)) {
      v = slipstream_receive (rx_buf);
      if (v > 0) {
	  gw_pkt.buf=rx_buf;
	  gw_pkt.buf_len=v;
	  print_gw_packet_elements(&gw_pkt); 
      }
      usleep (1000);
    }
    // What for NAV and service incoming messages 
    // This is the time window when the network is idle and can
    // be used for asynchronous communications.
    while (nav_timeout > time (NULL)) {
      v = slipstream_receive (rx_buf);
      if (v > 0) {
        // Check for mobile/p2p packets
	  gw_pkt.buf=rx_buf;
	  gw_pkt.buf_len=v;
	  print_gw_packet_elements(&gw_pkt); 
      }
      usleep (1000);
    }



  }
}

