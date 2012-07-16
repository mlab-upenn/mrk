#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <slipstream.h>

#define NONBLOCKING  0
#define BLOCKING     1


int main (int argc, char *argv[])
{
  char buffer[128];
  char buffer2[128];
  int v,cnt,i,corrupt;

  if (argc != 3) {
    printf ("Usage: server port\n");
    exit (1);
  }

  v=slipstream_open(argv[1],atoi(argv[2]),NONBLOCKING);
  printf( "Connecting to server: %s on port %d\n",argv[1], atoi(argv[2]));  

cnt=0;

while (1) {
    sprintf (buffer, "This is a sample slip string: Count %d\n", cnt);
    v=slipstream_send(buffer,strlen(buffer)+1);
    cnt++;
    if (v == 0) printf( "Error sending\n" );
    else printf( "Sent: %s",buffer );
    
    // Spin on reply message.  If you continue to send messages without a reply, the message will
    // queue up on the SLIPstream-server and become out of sync 
    do {    
    v=slipstream_receive( buffer2 );
    corrupt=0;
    if (v > 0) {
      printf ("Got: ");
      for (i = 0; i < v; i++)
      {
	      if(i<v && buffer[i]!=buffer2[i] )  corrupt=1;
              printf ("%c", buffer2[i]);
      }
    }
    if(corrupt==1) printf( "Packet Corrupt\n" );
    } while(v<=0);

    // Pause for 1 second 
    //sleep (1);
  }



}

