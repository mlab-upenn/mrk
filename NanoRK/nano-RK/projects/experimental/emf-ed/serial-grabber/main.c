#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

int main(void)
{
FILE *fp;
uint8_t c;
uint16_t data;
uint32_t cnt;

fp=fopen( "/dev/ttyUSB0", "r" );
if(fp==NULL )
{
	printf( "Couldn't open serial port\n" );
	exit(0);
}

printf( "Waiting for sync byte\n" );

cnt=0;
while(1)
{
c=0;
while(c!=0x55) c=fgetc(fp);
c=fgetc(fp);
data=c<<8;
c=fgetc(fp);
data|=c;
printf( "%u,%d\n",cnt, data);
cnt++;
}


}
