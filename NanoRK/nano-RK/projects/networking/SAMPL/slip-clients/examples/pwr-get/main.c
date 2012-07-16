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
#include <ping_pkt.h>
#include <pkt_debug.h>
#include <ff_basic_sensor_pkt.h>
#include <transducer_pkt.h>
#include <ff_power.h>
#include <transducer_registry.h>


//#include "tree_route.h"
//#include "slipstream.h"

#define gw_mac		0
#define CACHE_SIZE	32
uint8_t debug_txt_flag;
uint8_t subnet[3];
uint8_t dst_mac;
uint8_t ping_cache[CACHE_SIZE];
uint8_t ping_cache_index;

#define P2P_PKT		0
#define MULTICAST_PKT	1

#define NONBLOCKING  0
#define BLOCKING     1

#define HEX_STR_SIZE	5



uint8_t check_for_ack(uint8_t *buf,uint8_t v, uint8_t mac, uint8_t seq_num);
//uint8_t print_gw_packet_elements(uint8_t *buf, uint8_t len);

  int sockfd, portno, n;
  struct sockaddr_in serv_addr;
  struct hostent *server;
  int size;
  char buffer[2048];

SAMPL_DOWNSTREAM_PKT_T ds_pkt;
SAMPL_PEER_2_PEER_PKT_T p2p_pkt;
SAMPL_GATEWAY_PKT_T gw_pkt;

int main (int argc, char *argv[])
{
  FILE *fp;
  uint8_t tx_buf[128];
  uint8_t rx_buf[128];
  int32_t v,cnt,i,len,j;
  uint8_t nav_time_secs, reply_time_secs;
  uint8_t user_timeout,status,retry_cnt;
  int32_t tmp;
  time_t reply_timeout,nav_timeout,t;
  uint8_t cmd,error,led_debug,retry_num;
  char buf[1024];
  char mbuf[128];
  uint8_t num_msgs,mode;
  TRANSDUCER_PKT_T tran_pkt;
  TRANSDUCER_MSG_T tran_msg;
  FF_POWER_RQST_PKT     ff_pwr_rqst;
  uint8_t my_buf[32];
  uint8_t tran_pkt_buf[128];


debug_txt_flag=0;
led_debug=0;
mode=P2P_PKT;
user_timeout=5;
retry_num=0;

  if (argc < 4 ) {
    printf ("Usage: server port mac-addr [-t timeout] [-d]\n");
    printf ("  ex: %s localhost 5001 0x000000ff\n",argv[0]);
    printf ("  server		Address of SLIPstream server\n");
    printf ("  port		port of SLIPstream server or mirror\n");
    printf ("  mac-addr 	MAC address of node to actuate\n");
    printf ("  -t 		timeout in seconds (5 sec default)\n");
    printf ("  -d 		debug mode\n");
    printf ("  -l 		enable LED debug for multicast\n");
    exit (1);
  }

  for(i=0; i<argc; i++ )
	{
	// Grab dash command line options
	if(strstr(argv[i],"-d")!=NULL )
		debug_txt_flag=1;
	if(strstr(argv[i],"-l")!=NULL )
		led_debug=1;
	if(strstr(argv[i],"-r")!=NULL )
		{
		i++;
  		sscanf( argv[i],"%x",&tmp );
		retry_num=tmp;
		}	
	if(strstr(argv[i],"-t")!=NULL )
		{
		i++;
  		sscanf( argv[i],"%x",&tmp );
		user_timeout=tmp;
		}
	}

  	if(debug_txt_flag==1) printf( "Timeout: %d seconds\n",user_timeout);

  	sscanf( argv[3],"%x",&tmp );
	subnet[2]=(tmp&0xff000000) >> 24;
	subnet[1]=(tmp&0xff0000) >> 16;
	subnet[0]=(tmp&0xff00) >> 8;
	dst_mac=(tmp & 0xff);	

  	if(debug_txt_flag==1) printf( "MAC_ADDR: 0x%x\n",dst_mac);
	if(dst_mac==0xff) mode=MULTICAST_PKT;
	else mode=P2P_PKT;


  v=slipstream_open(argv[1],atoi(argv[2]),NONBLOCKING);
  nav_time_secs=25; 

  cnt = 0;
  error=0;
  cmd=0;


if(mode==MULTICAST_PKT)
{
	for(i=0; i<CACHE_SIZE; i++ ) ping_cache[i]=0;
	ping_cache_index=0;
if(retry_num==0) retry_num=1;
for(retry_cnt=0; retry_cnt<retry_num; retry_cnt++ )
{
  	if(debug_txt_flag==1) printf( "Building Multicast pkt\n");
	// These values setup the internal data structure and probably don't
	// need to be changed
	ds_pkt.payload_len=0;
	ds_pkt.buf=tx_buf;
	ds_pkt.buf_len=DS_PAYLOAD_START;
	ds_pkt.payload_start=DS_PAYLOAD_START;
	ds_pkt.payload=&(tx_buf[DS_PAYLOAD_START]);
	
	// These are parameters that can be adjusted for different packets
	ds_pkt.pkt_type=TRANSDUCER_PKT; 
	ds_pkt.ctrl_flags= DS_MASK | ENCRYPT; 
        if(led_debug==1) ds_pkt.ctrl_flags|= DEBUG_FLAG; 
	//srand(time(NULL));
	//ds_pkt.seq_num=rand()%255; // Use the gateway's spiffy auto-cnt when set to 0
	ds_pkt.seq_num=0; // Use the gateway's spiffy auto-cnt when set to 0
	ds_pkt.priority=0;
	ds_pkt.ack_retry=10;
	ds_pkt.subnet_mac[0]=subnet[0];
	ds_pkt.subnet_mac[1]=subnet[1];
	ds_pkt.subnet_mac[2]=subnet[2];
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

	// Add the packet to the payload  (ping is 0 size)
 
    tran_pkt.num_msgs = 0;
    tran_pkt.checksum = 0;
    tran_pkt.msgs_payload = tran_pkt_buf;


 	ff_pwr_rqst.socket=0;
        ff_pwr_rqst.pkt_type=DEBUG_PKT;

		// Request sensor values
		tran_msg.mac_addr = 0xff;
		tran_msg.type=TRAN_POWER_PKT;
		tran_msg.payload = mbuf;
		tran_msg.len=ff_power_rqst_pack(mbuf, &ff_pwr_rqst); 
		transducer_msg_add( &tran_pkt, &tran_msg );
						 

    ds_pkt.payload_len =
       transducer_pkt_pack(&tran_pkt,ds_pkt.payload);


        // This takes the structure and packs it into the raw
	// array that is sent using SLIP
        pack_downstream_packet( &ds_pkt);	

     // Add MAC filter entries below
     //  downstream_packet_add_mac_filter( &ds_pkt, 0x07 ); 
     //  downstream_packet_add_mac_filter( &ds_pkt, 3 ); 
     //  downstream_packet_add_mac_filter( &ds_pkt, 4 ); 
     //  downstream_packet_add_mac_filter( &ds_pkt, 5 ); 

    // Print your packet on the screen
    if(debug_txt_flag==1) printf( "Sent SLIP resquest\n" );
//	print_ds_packet(&ds_pkt );
    if(error==0) 
    	v=slipstream_send(ds_pkt.buf,ds_pkt.buf_len);
    reply_time_secs=ds_pkt.delay_per_level * ds_pkt.hop_max;

    reply_timeout=time(NULL)+reply_time_secs+user_timeout+2;
    while (reply_timeout > time (NULL)) {
      v = slipstream_receive (rx_buf);
      if (v > 0) {
		gw_pkt.buf=rx_buf;
		gw_pkt.buf_len=v;
		print_gw_packet_elements(&gw_pkt);	
		}

      usleep (1000);
    }
}

return status;
} 

if(mode==P2P_PKT){
if(retry_num==0) retry_num=3;
for(retry_cnt=0; retry_cnt<retry_num; retry_cnt++ )
{
  	if(debug_txt_flag==1) printf( "Building P2P pkt\n");
	// These values setup the internal data structure and probably don't
	// need to be changed
	p2p_pkt.payload_len=0;
	p2p_pkt.buf=tx_buf;
	p2p_pkt.buf_len=P2P_PAYLOAD_START;
	p2p_pkt.payload_start=P2P_PAYLOAD_START;
	p2p_pkt.payload=&(tx_buf[P2P_PAYLOAD_START]);
	
	// These are parameters that can be adjusted for different packets
	p2p_pkt.pkt_type=TRANSDUCER_PKT; 
	p2p_pkt.ctrl_flags= MOBILE_MASK | LINK_ACK | DEBUG_FLAG | ENCRYPT; 
	p2p_pkt.seq_num=0; // Use the gateway's spiffy auto-cnt when set to 0
	//p2p_pkt.seq_num=rand()%255; // Use the gateway's spiffy auto-cnt when set to 0
	p2p_pkt.priority=0;
	p2p_pkt.hop_cnt=0;
	p2p_pkt.ttl=5;
	p2p_pkt.ack_retry=10;
	p2p_pkt.src_subnet_mac[0]=subnet[0];
	p2p_pkt.src_subnet_mac[1]=subnet[1];
	p2p_pkt.src_subnet_mac[2]=subnet[2];
	p2p_pkt.src_mac=0;
	p2p_pkt.last_hop_mac=0;
	p2p_pkt.next_hop_mac=BROADCAST;
	p2p_pkt.dst_subnet_mac[0] = subnet[0];
    	p2p_pkt.dst_subnet_mac[1] = subnet[1];
    	p2p_pkt.dst_subnet_mac[2] = subnet[2];
    	p2p_pkt.dst_mac = dst_mac;
	p2p_pkt.check_rate=100;  // B-mac check rate in ms


	// At the top level you have a SAMPL packet (ds_pkt)
	// For transducers, this holds a single transducer pkt (tran_pkt)
	// Each transducer packet contains multiple transducer messages (tran_msg)
	// Transducers typically have helper functions used to pack the transducer messages.
  
    	tran_pkt.num_msgs = 0;
    	tran_pkt.checksum = 0;
    	tran_pkt.msgs_payload = tran_pkt_buf;


 	ff_pwr_rqst.socket=0;
        ff_pwr_rqst.pkt_type=DEBUG_PKT;

	// Request sensor values
	tran_msg.mac_addr = 0xff;
	tran_msg.type=TRAN_POWER_PKT;
	tran_msg.payload = mbuf;
	tran_msg.len=ff_power_rqst_pack(mbuf, &ff_pwr_rqst); 
	transducer_msg_add( &tran_pkt, &tran_msg );
	
						 

    p2p_pkt.payload_len =
       transducer_pkt_pack(&tran_pkt,p2p_pkt.payload);

  // This takes the structure and packs it into the raw
	// array that is sent using SLIP
        pack_peer_2_peer_packet( &p2p_pkt);	

    if(error==0) 
    	v=slipstream_send(p2p_pkt.buf,p2p_pkt.buf_len);

    if(debug_txt_flag==1)
    {
    if (v == 0) printf( "Error sending\n" );
    else printf( "Sent SLIP request\n");
    }
 

    reply_timeout=time(NULL)+user_timeout;
    while (reply_timeout > time (NULL)) {
      v = slipstream_receive (rx_buf);
      if (v > 0) {
		gw_pkt.buf=rx_buf;
		gw_pkt.buf_len=v;
		print_gw_packet_elements(&gw_pkt);
      }
      usleep (1000);
    }

    if(debug_txt_flag==1) printf( "Retry: %d\n",retry_cnt+1 );
}
return 0;

}
return 0;
}

/*
uint8_t print_gw_packet_elements(uint8_t *buf, uint8_t len)
{
PING_PKT_T ping;
SAMPL_GATEWAY_PKT_T gw_pkt;
uint8_t i,status,j,dup;
status=0;

gw_pkt.buf=buf;
gw_pkt.buf_len=len;
unpack_gateway_packet(&gw_pkt );
if(gw_pkt.pkt_type==PING_PKT)
	{
                for(i=0; i<gw_pkt.num_msgs; i++ )
                {
                ping_pkt_get(&ping, gw_pkt.payload, i );
		dup=0;
		for(j=0; j<ping_cache_index; j++ ) if(ping.mac_addr==ping_cache[j] ) dup=1;
		if(dup==0 ) 
			{
				printf( "Ack pkt from 0x%x\n",ping.mac_addr );
				ping_cache[ping_cache_index]=ping.mac_addr;
				ping_cache_index++;
			}
		status=1;
		}
	}

return status;
}

uint8_t check_for_ack(uint8_t *buf, uint8_t len, uint8_t mac, uint8_t seq_num)
{
PING_PKT_T ping;
SAMPL_GATEWAY_PKT_T gw_pkt;
uint8_t i;

gw_pkt.buf=buf;
gw_pkt.buf_len=len;
unpack_gateway_packet(&gw_pkt );
if(gw_pkt.pkt_type==PING_PKT)
	{
                for(i=0; i<gw_pkt.num_msgs; i++ )
                {
                ping_pkt_get(&ping, gw_pkt.payload, i );
		//printf( "Got ping pkt: %d\n",ping.mac_addr );
                if(ping.mac_addr==mac) return 1;
		}
	}

return 0;
}
*/
