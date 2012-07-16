#include <stdio.h>
#include <stdlib.h>

int main(void)
{
FILE *fp;
int mac_addr, outlet_num, outlet_state;

fp=fopen("/dev/ttyUSB0","w" );
if(fp==NULL)
{
printf( "Can't open com port\n" );
exit(-1);
}

mac_addr=1;
outlet_num=0;
outlet_state=0;
fprintf(fp, "S%c%c%cE",mac_addr, outlet_num, outlet_state );

fflush(fp);
fclose(fp);
return 1;
}
