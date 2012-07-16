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
#include <pkt_debug.h>
#include <ff_basic_sensor_pkt.h>
#include <transducer_pkt.h>
#include <ff_power.h>
#include <transducer_registry.h>
#include <ff_power.h>


//#include "tree_route.h"
//#include "slipstream.h"

#define gw_mac		0

uint8_t debug_txt_flag;
uint8_t subnet_3;
uint8_t subnet_2;
uint8_t subnet_1;

#define NONBLOCKING  0
#define BLOCKING     1

#define HEX_STR_SIZE	5



void error(char *msg);
void print_ds_packet(SAMPL_DOWNSTREAM_PKT_T *ds_pkt );
void print_gw_packet(SAMPL_GATEWAY_PKT_T *gw_pkt );

  int sockfd, portno, n;
  struct sockaddr_in serv_addr;
  struct hostent *server;
  int size;
  char buffer[2048];

SAMPL_DOWNSTREAM_PKT_T ds_pkt;

int main (int argc, char *argv[])
{
  FILE *fp;
  uint8_t tx_buf[128];
  uint8_t rx_buf[128];
  TRANSDUCER_PKT_T	tran_pkt;
  TRANSDUCER_MSG_T	tran_msg;
  int32_t v,cnt,i,len,j;
  uint8_t nav_time_secs, reply_time_secs;
  int32_t tmp;
  time_t reply_timeout,nav_timeout,t;
  uint8_t cmd,error;
  char buf[1024];
  char mbuf[128];
  uint8_t num_msgs;
  FF_POWER_RQST_PKT	ff_pwr_rqst;
  uint8_t ff_pwr_mac[10];
  SAMPL_GATEWAY_PKT_T gw_pkt;

debug_txt_flag=0;

  if (argc < 4 ) {
    printf ("Usage: server port num-msgs [mac-addr] [-d]\n");
    printf ("  ex: ./jigo-ctrl localhost 5000 1 0x000000f0\n");
    printf ("  mac-addr MAC address of node to request read\n");
    printf ("  d Debug Output\n");
    exit (1);
  }

  for(i=0; i<argc; i++ )
	{
	// Grab dash command line options
	if(strstr(argv[i],"-d")!=NULL )
		{	
		debug_txt_flag=1;
		}
	}

  sscanf( argv[3],"%d", &tmp);
	num_msgs=tmp;
  if(num_msgs>9 ) { printf( "Sorry, too many messages...\n" ); return 0; }
  if(debug_txt_flag==1)
  printf( "Composing %d actuation msgs\n", num_msgs );
  for(i=0; i<num_msgs; i++ )
  {
	sscanf( argv[i+4],"%x",&tmp );
	subnet_3=(tmp&0xff000000) >> 24;
	subnet_2=(tmp&0xff0000) >> 16;
	subnet_1=(tmp&0xff00) >> 8;
	ff_pwr_mac[i]=(tmp & 0xff);	
  	if(debug_txt_flag==1) printf( "MAC_ADDR: 0x%x ",ff_pwr_mac[i]);
  }

  v=slipstream_open(argv[1],atoi(argv[2]),NONBLOCKING);
  nav_time_secs=25; 

  cnt = 0;
  while (1) {
     error=0;
     cmd=0;


	// Setup the packet to send out to the network

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
	ds_pkt.ctrl_flags= DS_MASK | DEBUG_FLAG | ENCRYPT; 
	ds_pkt.seq_num=0; // Use the gateway's spiffy auto-cnt when set to 0
	ds_pkt.priority=0;
	ds_pkt.ack_retry=10;
	ds_pkt.subnet_mac[0]=subnet_1;
	ds_pkt.subnet_mac[1]=subnet_2;
	ds_pkt.subnet_mac[2]=subnet_3;
	ds_pkt.hop_cnt=0;  // Starting depth, always keep at 0
	ds_pkt.hop_max=5;  // Max tree depth
	ds_pkt.delay_per_level=1;  // Reply delay per level in seconds
	ds_pkt.nav=0;  // Time in seconds until next message to be sent
	ds_pkt.mac_check_rate=100;  // B-mac check rate in ms
	ds_pkt.rssi_threshold=-45;  // Reply RSSI threshold
	ds_pkt.last_hop_mac=0;
	ds_pkt.mac_filter_num=0; // Increase if MAC_FILTER is active
	ds_pkt.aes_ctr[0]=0;   // Encryption AES counter
	ds_pkt.aes_ctr[1]=0;
	ds_pkt.aes_ctr[2]=0;
	ds_pkt.aes_ctr[3]=0;


	// At the top level you have a SAMPL packet (ds_pkt)
	// For transducers, this holds a single transducer pkt (tran_pkt)
	// Each transducer packet contains multiple transducer messages (tran_msg)
	// Transducers typically have helper functions used to pack the transducer messages.
	
	// Setup Transducer Packet
	tran_pkt.num_msgs=0;	
	tran_pkt.checksum=0;	
	tran_pkt.msgs_payload=buf;	

	ff_pwr_rqst.socket=0;
	ff_pwr_rqst.pkt_type=DEBUG_PKT;

for(i=0; i<num_msgs; i++ )
{
	// Setup Transducer Message
	tran_msg.mac_addr = ff_pwr_mac[i];
	tran_msg.type = TRAN_POWER_PKT;
	tran_msg.len = 0;
	tran_msg.payload = mbuf;

	printf( "calling pack on mac %d\n",ff_pwr_mac[i] );
	// Pack application specifc message into transducer message
	tran_msg.len=ff_power_rqst_pack(mbuf, &ff_pwr_rqst);

	printf( "calling add\n" );
	// Add the transducer message to the transducer packet
	len=transducer_msg_add( &tran_pkt, &tran_msg);
	printf( "done with add payload len=%d\n",len );
//	printf( "payload = " );
//	for(j=0; j<len; j++ )
//		printf( "%d ",ds_pkt.payload[j] );

//	printf( "\n" );
}
    
	// Add the packet to the payload 
	ds_pkt.payload_len = transducer_pkt_pack(&tran_pkt, ds_pkt.payload); 

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


    cnt++;

    for(i=0; i<ds_pkt.buf_len; i++ )
    {
	printf( "%02x",ds_pkt.buf[i] );
    }
    printf( "\n" );
    if(error==0) 
    	v=slipstream_send(ds_pkt.buf,ds_pkt.buf_len);

    if(debug_txt_flag==1)
    {
    if (v == 0) printf( "Error sending\n" );
    else printf( "Sent request %d\n",ds_pkt.seq_num);
    }
 
    //nav_time_secs=ds_pkt.nav;
    nav_time_secs=5;
    reply_time_secs=ds_pkt.delay_per_level * ds_pkt.hop_max+3;

   t=time(NULL);
   reply_timeout=t+reply_time_secs+1;
   nav_timeout=t+nav_time_secs;

   // Collect Reply packets 
   while(reply_timeout>time(NULL))
   	{
    		v=slipstream_receive( rx_buf);
    		if (v > 0) {
    			gw_pkt.buf=rx_buf;
        		  gw_pkt.buf_len=v;
        		  print_gw_packet_elements(&gw_pkt);
	
		}
    		usleep(1000);
   	}

   // What for NAV and service incoming messages 
   // This is the time window when the network is idle and can
   // be used for asynchronous communications.
   while(nav_timeout>time(NULL))
   	{
    		v=slipstream_receive( rx_buf);
    		if (v > 0) {
      			// Check for mobile/p2p packets
    			gw_pkt.buf=rx_buf;
        		  gw_pkt.buf_len=v;
        		  print_gw_packet_elements(&gw_pkt);
    			}
    		usleep(1000);
   	}
	
	// only run once
	break;
}
}

