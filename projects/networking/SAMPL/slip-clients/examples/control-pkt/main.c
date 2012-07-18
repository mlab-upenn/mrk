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
#include <pkt_debug.h>

#define gw_mac		0

uint8_t debug_txt_flag;

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
  int32_t v,cnt,i,len;
  uint8_t nav_time_secs, reply_time_secs,checksum;
  int32_t tmp;
  time_t reply_timeout,nav_timeout,t;
  uint8_t cmd,error;
  char buf[1024];
  time_t clockt;
  uint32_t epocht;
  SAMPL_GATEWAY_PKT_T gw_pkt;

debug_txt_flag=0;

  if (argc < 3 || argc > 4) {
    printf ("Usage: server port [-d]\n");
    printf ("  d Debug Output\n");
    exit (1);
  }

  if(argc==4)
	{
	// Grab dash command line options
	if(strstr(argv[3],"d")!=NULL )
		{	
		debug_txt_flag=1;
		}
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
	ds_pkt.pkt_type=CONTROL_PKT; 
	ds_pkt.ctrl_flags= ENCRYPT | DS_MASK | LINK_ACK | DEBUG_FLAG; 
	ds_pkt.seq_num=0;
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
	ds_pkt.rssi_threshold=-40;  // Reply RSSI threshold
	ds_pkt.last_hop_mac=0;
	ds_pkt.mac_filter_num=0; // Increase if MAC_FILTER is active
	ds_pkt.aes_ctr[0]=0;   // Encryption AES counter
	ds_pkt.aes_ctr[1]=0;
	ds_pkt.aes_ctr[2]=0;
	ds_pkt.aes_ctr[3]=0;

	clockt = time(NULL);
    	epocht = clockt;
    	ds_pkt.epoch_time[0] = epocht & 0xff;      // Epoch time 
    	ds_pkt.epoch_time[1] = (epocht>>8) & 0xff;    
    	ds_pkt.epoch_time[2] = (epocht>>16) & 0xff;    
    	ds_pkt.epoch_time[3] = (epocht>>24) & 0xff;    



        ds_pkt.payload[0]=(-40);
	ds_pkt.payload[1]=0x01;
	ds_pkt.payload[2]=0x1f;
	ds_pkt.payload[3]=0xff;
	ds_pkt.payload[4]=0x1f;
	ds_pkt.payload[5]=0xff;
	checksum=0;
	for(i=0; i<6; i++ )
		checksum+=ds_pkt.payload[i];
	ds_pkt.payload[6]=checksum;  
	ds_pkt.payload_len=7;

        // This takes the structure and packs it into the raw
	// array that is sent using SLIP
        pack_downstream_packet( &ds_pkt);	

     // Add MAC filter entries below
     //  downstream_packet_add_mac_filter( &ds_pkt, 2 ); 
     //  downstream_packet_add_mac_filter( &ds_pkt, 3 ); 
     //  downstream_packet_add_mac_filter( &ds_pkt, 4 ); 
     //  downstream_packet_add_mac_filter( &ds_pkt, 5 ); 

    // Print your packet on the screen
    if(debug_txt_flag==1)
	print_ds_packet(&ds_pkt );


    cnt++;

    if(error==0) 
    	v=slipstream_send(ds_pkt.buf,ds_pkt.buf_len);

    if(debug_txt_flag==1)
    {
    if (v == 0) printf( "Error sending\n" );
    else printf( "Sent request %d\n",ds_pkt.seq_num);
    }
 
    nav_time_secs=ds_pkt.nav;
    reply_time_secs=ds_pkt.delay_per_level * ds_pkt.hop_max;

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
	


}
}

