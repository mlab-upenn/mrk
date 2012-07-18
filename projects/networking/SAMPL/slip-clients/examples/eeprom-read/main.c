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
#include <stats_pkt.h>
#include <pkt_debug.h>
#include <eeprom_data.h>

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

SAMPL_PEER_2_PEER_PKT_T p2p_pkt;
EEPROM_STORAGE_PKT_T data_pkt;

int main (int argc, char *argv[])
{
  FILE *fp;
  uint8_t tx_buf[128];
  uint8_t rx_buf[128];
  int32_t v,cnt,i,len;
  uint8_t nav_time_secs, reply_time_secs,checksum;
  time_t reply_timeout,nav_timeout,t;
  uint8_t cmd,error,echo;
  char buf[1024];
  time_t clockt;
  uint32_t epocht,tmp;
  uint8_t subnet_mac[3],node_mac;
  uint16_t eeprom_addr;
  uint8_t eeprom_size;
  SAMPL_GATEWAY_PKT_T gw_pkt;

debug_txt_flag=0;

  if (argc < 6 || argc > 7) {
    printf ("Usage: server port mac addr size [-v]\n");
    printf ("  -v verbose\n");
    printf ("  Example: ./eeprom-read localhost 5001 0x00000001 0x200 0x3d \n");
    exit (1);
  }

// Grab dash command line options
for(i=0; i<argc; i++ )
{
  if(strstr(argv[i],"-v")!=NULL)
	{
	printf( "Verbose enabled\n" );
	debug_txt_flag=1;
	}
}


  if (strlen (argv[3]) != 8 && strlen (argv[3]) != 10) {
    printf ("Invalid MAC address\n");
    exit(1);
  }
  v = sscanf (argv[3], "%x", &tmp);
  if (v != 1) {
    printf ("Invalid MAC address\n");
    exit(1);
  }
  subnet_mac[2]=(tmp&0xff000000) >> 24;
  subnet_mac[1]=(tmp&0xff0000) >> 16;
  subnet_mac[0]=(tmp&0xff00) >> 8;
  node_mac= tmp & 0xff;

  v = sscanf (argv[4], "%x", &tmp);
  if (v != 1) {
    printf ("Invalid EEPROM address\n");
    exit(1);
  }
  eeprom_addr=tmp&0xFFFF;
 
  v = sscanf (argv[5], "%x", &tmp);
  if (v != 1 || tmp>100) {
    printf ("Invalid EEPROM size\n");
    exit(1);
  }
  eeprom_size=tmp&0xFF;
 

  v=slipstream_open(argv[1],atoi(argv[2]),NONBLOCKING);

  nav_time_secs=25; 

  cnt = 0;
  while (1) {
     error=0;
     cmd=0;

	retry:
	// Setup the packet to send out to the network


  // These values setup the internal data structure and probably don't
        // need to be changed
        p2p_pkt.payload_len=0;
        p2p_pkt.buf=tx_buf;
        p2p_pkt.buf_len=P2P_PAYLOAD_START;
        p2p_pkt.payload_start=P2P_PAYLOAD_START;
        p2p_pkt.payload=&(tx_buf[P2P_PAYLOAD_START]);

        // These are parameters that can be adjusted for different packets
        //p2p_pkt.pkt_type=PING_PKT; 
        p2p_pkt.pkt_type=DATA_STORAGE_PKT;
        p2p_pkt.ctrl_flags= MOBILE_MASK | LINK_ACK | DEBUG_FLAG | ENCRYPT;
        p2p_pkt.seq_num=0; // Use the gateway's spiffy auto-cnt when set to 0
        //p2p_pkt.seq_num=rand()%255; // Use the gateway's spiffy auto-cnt when set to 0
        if(debug_txt_flag==1)
                printf( "rand seq-num=%d\n",p2p_pkt.seq_num );
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
        p2p_pkt.dst_subnet_mac[0] = subnet_mac[0];
        p2p_pkt.dst_subnet_mac[1] = subnet_mac[1];
        p2p_pkt.dst_subnet_mac[2] = subnet_mac[2];
        p2p_pkt.dst_mac = node_mac;
        p2p_pkt.check_rate=100;  // B-mac check rate in ms




	data_pkt.mode=EE_READ;
   	data_pkt.addr=eeprom_addr;  // Must be greater than 0x100
   	data_pkt.data_len=eeprom_size;


 p2p_pkt.payload_len =
	eeprom_storage_pkt_pack(&data_pkt, p2p_pkt.payload);
         // This takes the structure and packs it into the raw
        // array that is sent using SLIP
        pack_peer_2_peer_packet( &p2p_pkt);


    cnt++;

    if(error==0) 
    	v=slipstream_send(p2p_pkt.buf,p2p_pkt.buf_len);


    if(debug_txt_flag==1)
    {
    if (v == 0) printf( "Error sending\n" );
    else printf( "Sent request %d\n",p2p_pkt.seq_num);
    }
 
    nav_time_secs=10;

   t=time(NULL);
   nav_timeout=t+nav_time_secs;

   while(nav_timeout>time(NULL))
   	{
    		v=slipstream_receive( rx_buf);
    		if (v > 0) {
      			// Check for mobile/p2p packets
			gw_pkt.buf=rx_buf;
          		gw_pkt.buf_len=v;
	  		unpack_gateway_packet(&gw_pkt );
	  		if(gw_pkt.src_mac==node_mac && gw_pkt.dst_mac==0) print_gw_packet_elements(&gw_pkt);
    			}
    		usleep(1000);
   	}
	
return 1;

}
}

