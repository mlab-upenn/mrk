/******************************************************************************
 *  Sensor-Over-XMPP(SOX) tools
 *  Copyright (C) 2008, Real-Time and Multimedia Lab, Carnegie Mellon University
 *  All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, version 2.0 of the License.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *******************************************************************************/
#include <sensor_data.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

void sensor_data_init()
{
sensor_data_elements=0;
}

void sensor_data_print()
{
int i,j;
printf( "Sensor print data: %d\n",sensor_data_elements );
for(i=0; i<sensor_data_elements; i++ )
{
printf( "MAC:  0x%x ",sensor_cache[i].mac );
printf( "  Time: %s ",sensor_cache[i].time_str);
printf( "  DATA: 0x",sensor_cache[i].mac );
for(j=0; j<sensor_cache[i].data_size; j++ )
	{
	printf( "%02x",sensor_cache[i].data[j] );	
	}
printf( "\n" );
}


}

int8_t sensor_data_get(uint32_t mac, char *str)
{
int i,j;
uint16_t gas;
str[0]='\0';
for(i=0; i<sensor_data_elements; i++ )
{
if(sensor_cache[i].mac==mac )
	{
		sprintf( str,"<![CDATA[ ");
		sprintf( str,"%sTime: %s<br>",str,sensor_cache[i].time_str);
		for(j=0; j<7; j++ )
			{
			gas=sensor_cache[i].data[3+(j*2)]<<8 | sensor_cache[i].data[2+(j*2)];
			sprintf( str,"%sgas %d: %f<br>",str,j+1,((float)gas)/10);
			}
		sprintf( str,"%s ]]>",str);
	return 1;
	}
}

return 0;
}

int8_t sensor_data_add(uint32_t mac, char *str)
{
time_t timestamp;
int i,found,cnt,j;
char hex_byte[3];
found=0;
for(i=0; i<sensor_data_elements; i++ )
   {
	if(sensor_cache[i].mac==mac ) {
	found=1;
	break;
	}

   }

if(found==0)
{
if(sensor_data_elements>=MAX_SENSOR_NODES ) return -1;
sensor_cache[sensor_data_elements].mac=mac;
i=sensor_data_elements;
sensor_data_elements++;
}

	time (&timestamp);
        strftime (&(sensor_cache[i].time_str), 100, "%Y-%m-%d %X", localtime (&timestamp));
cnt=0;
for(j=0; j<strlen(str); j+=2 )
{
hex_byte[0]=str[j];
hex_byte[1]=str[j+1];
hex_byte[2]='\0';
sscanf(hex_byte,"%x",&(sensor_cache[i].data[cnt]) );
cnt++;
}

	sensor_cache[i].data_size=cnt;
return 1;
}

