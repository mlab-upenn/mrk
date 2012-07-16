#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <slipstream.h>
#include <pkt.h>
#include <time.h>


#define NONBLOCKING  0
#define BLOCKING     1


int orb_set (uint32_t mac, uint8_t color, uint8_t pulse);

int send_ping ();

int verbose;

int main (int argc, char *argv[])
{
  char buffer[128];
  int v, cnt, i, size, color, pulse;
  uint32_t target_mac;
  time_t ts;

  if (argc < 6) {
    printf ("Usage: server port mac-addr color animation [-v]\n");
    printf ("  mac-addr:  mac address in hex\n");
    printf ("  color:     0-36\n");
    printf ("  animation: 0-9\n");
    printf ( "\n\nCOLORS:\n" );
    printf ( "   RED->0,ORANGE->3,YELLOW->6,GREEN->12,AQUA->16,CYAN->18,BLUE->24,\n" );
    printf ( "   VIOLET->27,PURPLE->28,MAGENTA->30\n" );
    printf ( "\nANIMATIONS:\n" );
    printf ( "   NONE->0,VERY_SLOW->1,SLOW->2,MEDIUM_SLOW->3,MEDIUM->4,MEDIUM_FAST->5,FAST->6,\n" );
    printf ( "   VERY_FAST->7,CRESCENDO->8,HEARTBEAT->9\n" );

    printf ("\n\n  Example:   ./orb-ctrl localhost 5000 0 0\n");
    exit (1);
  }

  color = 0;
  pulse = 0;
  verbose=0;
	if(argc==7)
	{
		if(strcmp(argv[6],"-v")==0) { printf( "Verbose on\n" ); verbose=1; }
	}

	sscanf( argv[3],"%x",&target_mac );
	color=atoi(argv[4]);
	pulse=atoi(argv[5]);

  v = slipstream_open (argv[1], atoi (argv[2]), BLOCKING);


  if(verbose) printf ("Sending an actuate command [mac=0x%x color %d, pulse %d]\n",target_mac,color,pulse);
  // Enable outlet 0 on node 0x01
  orb_set (target_mac, color, pulse);



}



int orb_set (uint32_t mac, uint8_t color, uint8_t pulse)
{
  int v, size;
  PKT_T my_pkt;
  char buffer[128];

  my_pkt.src_mac = 0x00000000;
  my_pkt.dst_mac = mac;
  my_pkt.type = APP;
  my_pkt.payload[0] = 1;        // Number of Elements
  my_pkt.payload[1] = 3;        // KEY (3->orb ) 
  my_pkt.payload[2] = color;  
  my_pkt.payload[3] = pulse;    
  my_pkt.payload_len = 4;
  size = pkt_to_buf (&my_pkt, buffer);
  v = slipstream_send (buffer, size);
  return v;
}

