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
#include <ack_pkt.h>
#include <ff_basic_sensor_pkt.h>
#include <transducer_pkt.h>
#include <transducer_registry.h>

#define NONBLOCKING 0
#define BLOCKING 1

#define HEX_STR_SIZE 5

uint8_t check_for_ack(uint8_t *buf,uint8_t v, uint8_t mac, uint8_t seq_num);

uint8_t debug_txt_flag;
uint8_t subnet_3;
uint8_t subnet_2;
uint8_t subnet_1;

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
  TRANSDUCER_PKT_T	tran_pkt;
  TRANSDUCER_MSG_T	tran_msg;
  int32_t v,cnt,i,len,j;
  uint8_t nav_time_secs, reply_time_secs;
  int32_t tmp;
  time_t reply_timeout,nav_timeout,t;
  uint8_t cmd,error;
  char buf[1024];
  char mbuf[128];
  uint8_t p2p_mode, retry_flag;
  uint8_t node_mac_addr;
  char* lcd_msg; // Message to send to LCD

  debug_txt_flag=0;
  p2p_mode=1;
  retry_flag=0;

  if(argc < 5 || argc > 7)
  {
    printf("Usage: %s server port mac-addr \'message\' [-p] [-d]\n", argv[0]);
    printf("Example: %s localhost 5000 0x00000001 \'Hello\\nWorld!\' -p\n", argv[0]);
    printf("  mac-addr     MAC address of node to display LCD message\n");
    printf("  -p           Use P2P packet mode\n");
    printf("  -m           Multi-cast\n");
    printf("  -d           Display debugging output\n");
    printf("  -r           Local Retry, don't use with slip mirror gateway\n");
    printf("  -l           Use Link ACK routing\n");
    exit(1);
  }

  // Grab dash command line options
  if(argc > 5)
  {
    for(i = 5; i < argc; i++)
    {
      if(strcmp(argv[i], "-p") == 0)
      {
        p2p_mode = 1;
        printf("*P2P Mode Enabled\n");
      }
      if(strcmp(argv[i], "-m") == 0)
      {
        p2p_mode = 0;
        printf("*Multi-cast Mode Enabled\n");
      }
       else if(strcmp(argv[i], "-d") == 0)
      {
        debug_txt_flag = 1;
        printf("*Debug Mode Enabled\n");
      }
        else if(strcmp(argv[i], "-r") == 0)
      {
        retry_flag= 1;
        printf("*Retry Enabled\n");
      }
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
  subnet_3=(tmp&0xff000000) >> 24;
  subnet_2=(tmp&0xff0000) >> 16;
  subnet_1=(tmp&0xff00) >> 8;
  node_mac_addr = tmp & 0xff;

  v = slipstream_open (argv[1], atoi (argv[2]), NONBLOCKING);
  
  // Set pointer to message
  lcd_msg = (char*)argv[4];

  nav_time_secs = 25;

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
	ds_pkt.ctrl_flags= DS_MASK | DEBUG_FLAG | ENCRYPT; 
	//srand(time(NULL));
	//ds_pkt.seq_num=rand()%255; // Use the gateway's spiffy auto-cnt when set to 0
	ds_pkt.seq_num=0; // Use the gateway's spiffy auto-cnt when set to 0
        if(debug_txt_flag==1)
		printf( "rand seq-num=%d\n",ds_pkt.seq_num );
	ds_pkt.priority=0;
	ds_pkt.ack_retry=10;
	ds_pkt.subnet_mac[0]=subnet_1;
	ds_pkt.subnet_mac[1]=subnet_2;
	ds_pkt.subnet_mac[2]=subnet_3;
	ds_pkt.hop_cnt=0;  // Starting depth, always keep at 0
	ds_pkt.hop_max=7;  // Max tree depth
	ds_pkt.delay_per_level=0;  // Reply delay per level in seconds
	ds_pkt.nav=0;  // Time in seconds until next message to be sent
	ds_pkt.mac_check_rate=100;  // B-mac check rate in ms
	ds_pkt.rssi_threshold=-35;  // Reply RSSI threshold
	ds_pkt.last_hop_mac=0;
	ds_pkt.mac_filter_num=0; // Increase if MAC_FILTER is active
	ds_pkt.aes_ctr[0]=0;   // Encryption AES counter
	ds_pkt.aes_ctr[1]=0;
	ds_pkt.aes_ctr[2]=0;
	ds_pkt.aes_ctr[3]=0;
} else
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
//	p2p_pkt.ctrl_flags= MOBILE_MASK | /*LINK_ACK |*/ DEBUG_FLAG | ENCRYPT; 
	p2p_pkt.ctrl_flags= MOBILE_MASK | LINK_ACK | DEBUG_FLAG | ENCRYPT; 
	//p2p_pkt.ctrl_flags= MOBILE_MASK | DEBUG_FLAG | ENCRYPT; 
	p2p_pkt.seq_num=0; // Use the gateway's spiffy auto-cnt when set to 0
	//p2p_pkt.seq_num=rand()%255; // Use the gateway's spiffy auto-cnt when set to 0
        if(debug_txt_flag==1)
		printf( "rand seq-num=%d\n",p2p_pkt.seq_num );
	p2p_pkt.priority=0;
	p2p_pkt.hop_cnt=0;
	p2p_pkt.ttl=7;
	p2p_pkt.ack_retry=10;
	p2p_pkt.src_subnet_mac[0]=0;
	p2p_pkt.src_subnet_mac[1]=0;
	p2p_pkt.src_subnet_mac[2]=0;
	p2p_pkt.src_mac=0;
	p2p_pkt.last_hop_mac=0;
	p2p_pkt.next_hop_mac=BROADCAST;
	p2p_pkt.dst_subnet_mac[0] = 0;
    	p2p_pkt.dst_subnet_mac[1] = 0;
    	p2p_pkt.dst_subnet_mac[2] = 0;
    	p2p_pkt.dst_mac = node_mac_addr;
	p2p_pkt.check_rate=100;  // B-mac check rate in ms


}

	// At the top level you have a SAMPL packet (ds_pkt)
	// For transducers, this holds a single transducer pkt (tran_pkt)
	// Each transducer packet contains multiple transducer messages (tran_msg)
	// Transducers typically have helper functions used to pack the transducer messages.
	
	// Setup Transducer Packet
	tran_pkt.num_msgs=0;	
	tran_pkt.checksum=0;	
	tran_pkt.msgs_payload=buf;

        // Build message to send to LCD
        tran_msg.mac_addr = node_mac_addr;
        tran_msg.type = TRAN_LCD_MESSAGE;
        tran_msg.len = strlen(lcd_msg) + 1;
        for(i = 0; i < tran_msg.len; i++)
          mbuf[i] = (uint8_t)lcd_msg[i];
        tran_msg.payload = mbuf;

        // Add the transducer message to the transducer packet
        transducer_msg_add( &tran_pkt, &tran_msg);
   
if(!p2p_mode)
{
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
    if(error==0)
    {
    	v=slipstream_send(ds_pkt.buf,ds_pkt.buf_len);
      for(i=0; i<100; i++ )
      {
        v = slipstream_receive (rx_buf);
        if(v==1 && rx_buf[0]=='A' ) { printf( "Serial Good\n" ); break;}
        if(v==1 && rx_buf[0]=='N') {
			printf( "Serial Error 1\n" );
			}
	usleep(1000);
      }
      if(i==100) printf( "Serial Error 2\n" );
    }
    //nav_time_secs=ds_pkt.nav;
    nav_time_secs=5;
    reply_time_secs=ds_pkt.delay_per_level * ds_pkt.hop_max;
}
else {
    p2p_pkt.payload_len =
      transducer_pkt_pack(&tran_pkt,p2p_pkt.payload);
         // This takes the structure and packs it into the raw
	// array that is sent using SLIP
        pack_peer_2_peer_packet( &p2p_pkt);	

    if(error==0) 
    {
	    if(retry_flag)
    	v=slipstream_acked_send(p2p_pkt.buf,p2p_pkt.buf_len,3);
    	else 
		v=slipstream_send(p2p_pkt.buf,p2p_pkt.buf_len);
    }
  


    if(debug_txt_flag==1)
    {
    if (v == 0) printf( "Serial Error\n" );
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

    //LCD message sent
    if(v > 0)
      printf("[%i] Message \'%s\' sent to node 0x%x.\n", cnt, lcd_msg, node_mac_addr);

    t=time(NULL);
    reply_timeout=t+reply_time_secs+1;
    nav_timeout=t+nav_time_secs;


    while (nav_timeout > time (NULL)) {
      v = slipstream_receive (rx_buf);
      if (v > 0) {
	if (check_for_ack(rx_buf,v, node_mac_addr, p2p_pkt.seq_num)==1 ) { printf( "ACK\n" ); return 1; }
      }
      usleep (1000);
    }

    cnt++;
    if(cnt>2) break;
  }
printf( "NCK\n" );
return 0;
}


uint8_t check_for_ack(uint8_t *buf, uint8_t len, uint8_t mac, uint8_t seq_num)
{
SAMPL_GATEWAY_PKT_T gw_pkt;
TRANSDUCER_PKT_T tran_pkt;
TRANSDUCER_MSG_T tran_msg;
uint8_t i;

gw_pkt.buf=buf;
gw_pkt.buf_len=len;
unpack_gateway_packet(&gw_pkt );

if(gw_pkt.pkt_type==TRANSDUCER_PKT)
	{
                transducer_pkt_unpack(&tran_pkt, gw_pkt.payload );
                for(i=0; i<tran_pkt.num_msgs; i++ )
                {
                transducer_msg_get(&tran_pkt, &tran_msg, i );
                if(tran_msg.type==TRAN_ACK)
			{
			if(tran_msg.mac_addr==mac ) 
				return 1;
			}
		}
	}

return 0;
}
