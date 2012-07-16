#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#define BUF_SIZE	1000
#define DC_OFFSET	512

int min_max_buf[100];
int buf[BUF_SIZE];
uint32_t rms_cnt,cnt;
int min, max, thresh;
int str_nxt_min, str_nxt_max;

void event_detector_init()
{
int i;
	str_nxt_min=-1;
	str_nxt_max=-1;
	rms_cnt=0;
	cnt=0;
	for(i=0; i<BUF_SIZE; i++ ) buf[i]=0;

}

uint16_t event_detector(int16_t value)
{
double rms;
uint32_t i;
uint8_t state;
if(value>1024) value=1024;
if(value<0) value=0;
value=value-DC_OFFSET;
if(value<0) value*=-1;
buf[cnt]=value;
cnt++;
if(cnt>=BUF_SIZE)
	{
	rms=0;
	for(i=0; i<BUF_SIZE; i++ )
		rms+=buf[i]*buf[i];
		rms/=BUF_SIZE;
	rms=sqrt(rms);


	if(rms_cnt<100) {
		if(rms<min && str_nxt_min==-1) str_nxt_min=5;
		if(rms>max && str_nxt_max==-1) str_nxt_max=5;
		if(str_nxt_max>0) str_nxt_max--;
		if(str_nxt_max==0) {
			if(rms>max) max=rms;
			str_nxt_max=-1;
		}
		if(str_nxt_min>0) str_nxt_min--;
		if(str_nxt_min==0) {
			if(rms<min) min=rms;
			str_nxt_min=-1;
		}
		printf( "Training[%d] min: %d max: %d ",100-rms_cnt, min,max );
		thresh=(max-min)/2 + min;
	}

	
	if(rms>thresh)state=1;
	else state=0;

	printf( "%d,%d,%d\n",rms_cnt,(unsigned int)rms,state );

	rms_cnt++;
	cnt=0;
	}
return((uint16_t)rms);
}

int main(void)
{
FILE *fp;
uint8_t c;
uint16_t data,last_rms;

fp=fopen( "/dev/ttyUSB0", "r" );
if(fp==NULL )
{
	printf( "Couldn't open serial port\n" );
	exit(0);
}

event_detector_init();
printf( "Waiting for sync byte\n" );

min=1024;
max=0;
while(1)
{
c=0;
while(c!=0x55) c=fgetc(fp);
c=fgetc(fp);
data=c<<8;
c=fgetc(fp);
data|=c;
last_rms=event_detector(data);
//printf( "%u,%d\n",cnt, data);
}


}
