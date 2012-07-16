#include <slip-server.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdlib.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdlib.h>


static struct hostent *hp;
static int sock, length, fromlen,replylen, n,reply_sock;
static struct sockaddr_in server;
static struct sockaddr_in from;
static struct sockaddr_in client;
static int got_connection;
static int reply_mode;
static int debug;


void slipstream_server_open (int port, char *client_ip, uint8_t debug_flag)
{
  reply_mode=STATIC_CLIENT;
  debug=debug_flag;
  got_connection = 0;
  sock = socket (AF_INET, SOCK_DGRAM, 0);
  // Non-blocking socket
  fcntl (sock, F_SETFL, O_NONBLOCK);
  if (sock < 0)
    perror ("Opening socket");
  length = sizeof (server);
  bzero (&server, length);
  server.sin_family = AF_INET;
  server.sin_addr.s_addr = INADDR_ANY;
  server.sin_port = htons (port);
  if (bind (sock, (struct sockaddr *) &server, length) < 0)
    perror ("binding");
  fromlen = sizeof (struct sockaddr_in);

  replylen = sizeof (struct sockaddr_in);

if(reply_mode==STATIC_CLIENT)
  {
        if(debug==1) printf( "Setting up static client\r\n" );

        client.sin_family = AF_INET;
        hp = gethostbyname (client_ip);
        if (hp == 0)
          {
                perror ("Unknown client host");
                return;
          }

        bcopy ((char *) hp->h_addr, (char *) &client.sin_addr, hp->h_length);
        client.sin_port = htons (port);

  }


}

uint8_t slipstream_server_tx( uint8_t *buf, uint8_t size)
{
int i;

  if (debug==1) {
    printf ("\nTX [");
    for (i = 0; i < size; i++) {
      printf ("%x ", buf[i]);
    }
    printf ("]\n");
  }

  if (got_connection != 0) {
        n = sendto (sock, buf, size, 0, (struct sockaddr *) &client, replylen);
        if (n < 0)
		{
                	perror ("sendto");
			return 0;
		}
   } else return 0;

return 1;
}

uint8_t slipstream_server_non_blocking_rx (uint8_t *buf)
{
  int i;

  n = recvfrom (sock, buf, 1024, 0, (struct sockaddr *) &from, &fromlen);

  if (n > 0) {
  if(reply_mode==STATIC_CLIENT)
        {
        if(from.sin_addr.s_addr != client.sin_addr.s_addr)
                {
                printf( "Reject packet\r\n" );
                return 0;
                }
        }
    bcopy(&from,&client,sizeof(struct sockaddr));
    got_connection = 1;
    if (debug==1) {
      printf ("\nRX [");
      for (i = 0; i < n; i++)
        printf ("%x ", buf[i]);
      printf ("]\n");
    }
    return n;
  }
return 0;
}

