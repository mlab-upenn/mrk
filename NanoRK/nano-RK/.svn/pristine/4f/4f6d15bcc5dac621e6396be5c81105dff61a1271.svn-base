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


//#include "tree_route.h"
//#include "slipstream.h"

#define gw_mac		0

uint8_t debug_txt_flag;

#define NONBLOCKING  0
#define BLOCKING     1

#define HEX_STR_SIZE	5


  int sockfd, portno, n;
  struct sockaddr_in serv_addr;
  struct hostent *server;
  int size;
  char buffer[2048];

SAMPL_DOWNSTREAM_PKT_T ds_pkt;
SAMPL_PEER_2_PEER_PKT_T p2p_pkt;

int main (int argc, char *argv[])
{
  FILE *fp;
  uint8_t tx_buf[128];
  uint8_t rx_buf[128];
  int32_t v,cnt,i,len;
  uint8_t nav_time_secs, reply_time_secs;
  int32_t tmp;
  time_t reply_timeout,nav_timeout,t;
  uint8_t cmd,error,echo;
  char buf[1024];
  uint8_t my_buf[32];
  uint8_t tran_pkt_buf[128];
  uint8_t p2p_mode;
  TRANSDUCER_PKT_T tran_pkt;
  TRANSDUCER_MSG_T tran_msg;
  SAMPL_GATEWAY_PKT_T gw_pkt; 

p2p_mode=0;
debug_txt_flag=0;

  if (argc < 3 || argc > 5) {
    printf ("Usage: server port [-d] [-p]\n");
    printf ("  -d Debug Output\n");
    printf ("  -p P2P packet mode instead of DS\n");
    exit (1);
  }

  for(i=0; i<argc; i++ )
	{
	// Grab dash command line options
	if(strcmp(argv[i],"-d")==0)
		debug_txt_flag=1;
	if(strcmp(argv[i],"-p")==0)
		p2p_mode=1;
	}


  v=slipstream_open(argv[1],atoi(argv[2]),NONBLOCKING);
  nav_time_secs=25; 

if(p2p_mode)
 printf( "Running in P2P packet mode\n" );
else  printf( "Running in Downstream packet mode\n" );


  cnt = 0;
  while (1) {
     error=0;
     cmd=0;

     retry:

	// Setup the packet to send out to the network

if(!p2p_mode)
{
	// These values setup the internal data structure and probably don't
	// need to be changed
	ds_pkt.payload_len=0;
	ds_pkt.buf=tx_buf;
	ds_pkt.buf_len=DS_PAYLOAD_START;
	ds_pkt.payload_start=DS_PAYLOAD_START;
	ds_pkt.payload=&(tx_buf[DS_PAYLOAD_START]);
	
	// These are parameters that can be adjusted for different packets
	//ds_pkt.pkt_type=PING_PKT; 
	ds_pkt.pkt_type=TRANSDUCER_PKT; 
	ds_pkt.ctrl_flags= DS_MASK | LINK_ACK | DEBUG_FLAG ; 
	ds_pkt.seq_num=cnt;
	ds_pkt.priority=0;
	ds_pkt.ack_retry=10;
	ds_pkt.subnet_mac[0]=0;
	ds_pkt.subnet_mac[1]=0;
	ds_pkt.subnet_mac[2]=0;
	ds_pkt.hop_cnt=0;  // Starting depth, always keep at 0
	ds_pkt.hop_max=5;  // Max tree depth
	ds_pkt.delay_per_level=1;  // Reply delay per level in seconds
	ds_pkt.nav=30;  // Time in seconds until next message to be sent
	ds_pkt.mac_check_rate=100;  // B-mac check rate in ms
	ds_pkt.rssi_threshold=-45;  // Reply RSSI threshold
	ds_pkt.last_hop_mac=0;
	ds_pkt.mac_filter_num=0; // Increase if MAC_FILTER is active
}       
else
{
	// These values setup the internal data structure and probably don't
	// need to be changed
	p2p_pkt.payload_len=0;
	p2p_pkt.buf=tx_buf;
	p2p_pkt.buf_len=P2P_PAYLOAD_START;
	p2p_pkt.payload_start=P2P_PAYLOAD_START;
	p2p_pkt.payload=&(tx_buf[P2P_PAYLOAD_START]);
	
	// These are parameters that can be adjusted for different packets
	//p2p_pkt.pkt_type=PING_PKT; 
	p2p_pkt.pkt_type=TRANSDUCER_PKT; 
	p2p_pkt.ctrl_flags= MOBILE_MASK | LINK_ACK | DEBUG_FLAG | ENCRYPT; 
	p2p_pkt.seq_num=cnt;
	p2p_pkt.priority=0;
	p2p_pkt.hop_cnt=0;
	p2p_pkt.ttl=5;
	p2p_pkt.ack_retry=10;
	p2p_pkt.src_subnet_mac[0]=0;
	p2p_pkt.src_subnet_mac[1]=0;
	p2p_pkt.src_subnet_mac[2]=0;
	p2p_pkt.src_mac=0;
	p2p_pkt.last_hop_mac=0;
	p2p_pkt.next_hop_mac=BROADCAST;
	p2p_pkt.dst_subnet_mac[0] = BROADCAST;
    	p2p_pkt.dst_subnet_mac[1] = BROADCAST;
    	p2p_pkt.dst_subnet_mac[2] = BROADCAST;
    	p2p_pkt.dst_mac = BROADCAST;
	p2p_pkt.check_rate=100;  // B-mac check rate in ms
 }       



 

        tran_pkt.num_msgs = 0;
    	tran_pkt.checksum = 0;
    	tran_pkt.msgs_payload = tran_pkt_buf;


    	tran_msg.mac_addr = 0xff;
    	tran_msg.type=TRAN_FF_BASIC_SHORT;
    	tran_msg.len= 0;
    		//   my_buf[0]=1;   
    	tran_msg.payload = my_buf;

    	transducer_msg_add( &tran_pkt, &tran_msg );

if(!p2p_mode)
{
    ds_pkt.payload_len =
      transducer_pkt_pack(&tran_pkt,ds_pkt.payload);
    printf( "payload len= %d\n",ds_pkt.payload_len );

         // This takes the structure and packs it into the raw
	// array that is sent using SLIP
        pack_downstream_packet( &ds_pkt);	

     // Add MAC filter entries below
     //  downstream_packet_add_mac_filter( &ds_pkt, 0x07 ); 
     //  downstream_packet_add_mac_filter( &ds_pkt, 3 ); 
     //  downstream_packet_add_mac_filter( &ds_pkt, 4 ); 
     //  downstream_packet_add_mac_filter( &ds_pkt, 5 ); 

    // Print your packet on the screen
    if(debug_txt_flag==1)
	print_ds_packet(&ds_pkt );

    nav_time_secs=ds_pkt.nav;
    reply_time_secs=ds_pkt.delay_per_level * ds_pkt.hop_max;


    if(error==0) 
    	v=slipstream_send(ds_pkt.buf,ds_pkt.buf_len);
}
else
{
    p2p_pkt.payload_len =
      transducer_pkt_pack(&tran_pkt,p2p_pkt.payload);
    printf( "payload len= %d\n",p2p_pkt.payload_len );
         // This takes the structure and packs it into the raw
	// array that is sent using SLIP
        pack_peer_2_peer_packet( &p2p_pkt);	

    if(error==0) 
    	v=slipstream_send(p2p_pkt.buf,p2p_pkt.buf_len);

    if(debug_txt_flag==1)
    {
    if (v == 0) printf( "Error sending\n" );
    else printf( "Sent request %d\n",p2p_pkt.seq_num);
    }
 
    nav_time_secs=5;
    reply_time_secs=5;
}



    if(debug_txt_flag==1)
    {
    if (v == 0) printf( "Error sending\n" );
    else printf( "Sent request %d\n",cnt);
    }
    cnt++;
 

   t=time(NULL);
   reply_timeout=t+reply_time_secs+1;
   nav_timeout=t+nav_time_secs;

   echo=0;

    // Collect Reply packets 
    while (reply_timeout > time (NULL)) {
      v = slipstream_receive (rx_buf);
      if (v > 0) {
        if (echo == 0) {
          int ef;
          ef = 0;
          for (i = 0; i < ds_pkt.buf_len; i++)
            if (rx_buf[i] != ds_pkt.buf[i]) {
              ef = 1;
              printf ("Error %d:  %d!=%d\n", i, ds_pkt.buf[i], rx_buf[i]);
            }
          if (ef == 0)
            printf ("Gateway packet returned correctly.\n");
          //else exit(0);
          echo = 1;
        }
        else
	{
	gw_pkt.buf=rx_buf;
          gw_pkt.buf_len=v;
          print_gw_packet_elements(&gw_pkt);

	}
      }
      usleep (1000);
    }


    if (echo == 0) {
      printf ("Error, no serial reply from gateway!\n");
      goto retry;
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

