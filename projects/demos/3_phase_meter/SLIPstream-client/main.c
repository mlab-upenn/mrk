#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <slipstream.h>

#define NONBLOCKING  0
#define BLOCKING     1

#define PwrMul_fac 3.44
#define PwrMul_exp 0.0001

#define CurrMul_fac 1.244
#define CurrMul_exp 0.000001

#define VoltMul_fac 3.296
#define VoltMul_exp 0.00001


FILE *WrRawData, *WrRealData, *WrFlData;
int main (int argc, char *argv[])
{
	char buffer[128];
	int v,cnt,i;
	signed int watt, var;
	float watt_f,var_f;

	if (argc != 3) 
	{
		printf ("Usage: server port\n");
		exit (1);
	}
	WrRawData = fopen("Rawdata.txt","wb");
	WrRealData = fopen("Realdata.txt","wb");
	WrFlData = fopen("Fldata.txt","wb");

	v=slipstream_open(argv[1],atoi(argv[2]),NONBLOCKING);

	sprintf (buffer, "This is a sample slip string: Count %d\n", cnt);

	v=slipstream_send(buffer,strlen(buffer)+1);
	cnt = 0;
	while (1) 
	{
		cnt++;
		//sprintf (buffer, "This is a sample slip string: Count %d\n", cnt);

		//v=slipstream_send(buffer,strlen(buffer)+1);
		//if (v == 0) printf( "Error sending\n" );

		v=slipstream_receive( buffer );
		if (v > 0) 
		{
			printf ("Got: ");
			for (i = 0; i < v; i++)
				printf ("%x ", (uint8_t)buffer[i]);
			printf ("\n");
		}

		//Added in V2 - printing to file
		if (v > 0) 
		{
			fprintf(WrRawData,"Got: ");
			for (i = 0; i < v; i++)
				fprintf(WrRawData,"%x ", (uint8_t)buffer[i]);
			fprintf(WrRawData,"\n");
		}
		//Added in V3 - printing to file
		if (v > 0) 
		{
			//fprintf(WrRealData,"Got: ");
			for (i = 0; i < v; i=i+6)
			{
				watt = 0;var = 0;
				memcpy(&watt,buffer+i,3);
				if((watt&0x800000)>>23 ==1)
					watt |=0xff000000;
				watt_f = ((float)watt)*VoltMul_exp;
				watt_f = watt_f*VoltMul_fac;
				memcpy(&var,buffer+i+3,3);
				if((var&0x800000)>>23 ==1)
					var |=0xff000000;	
				var_f = ((float)var)*CurrMul_exp;
				//var_f = var_f*1.398; not sure why 1.398 was there
				var_f = var_f*CurrMul_fac;
				fprintf(WrRealData,"\n#\n%08d\n%08d",watt,var);
				fprintf(WrFlData,"\n#\n%04f\n%04f",watt_f,var_f);
			}
		}
	// Pause for 1 second 
	// sleep (1);
	}
}

