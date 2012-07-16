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
#include <ff_power.h>
#include <transducer_registry.h>
#include <pkt_debug.h>


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

  if (argc < 2 ) {
    printf ("Usage: server port [-d]\n");
    printf ("  ex: %s localhost 5000\n",argv[0]);
    printf ("  d Debug Output\n");
    exit (1);
  }

  for(i=0; i<argc; i++ )
	{
	// Grab dash command line options
	if(strstr(argv[i],"-d")!=NULL )
		{	
		debug_txt_flag=1;
		printf( "Debug Messages enabled\n" );
		}
	}

  v=slipstream_open(argv[1],atoi(argv[2]),NONBLOCKING);

  v = slipstream_send (rx_buf, 1);
  while (1) {

    		v=slipstream_receive( rx_buf);
    		if (v > 0) {
    		gw_pkt.buf=rx_buf;
          gw_pkt.buf_len=v;
          print_gw_packet_elements(&gw_pkt);
	
		}
		usleep(100);
	}
}

