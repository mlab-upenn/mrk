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
#include <loc_centroid.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>


void loc_beacon_print()
{
int i,j;

printf( "AP list:\n" );
for(i=0; i<ap_num; i++ )
{
printf( "MAC:  0x%x ",ap[i].mac ); 
printf( "Coord: %d, %d ",ap[i].x,ap[i].y);
printf( "Desc: %s\n",ap[i].desc); 
}


}

void loc_beacon_load(char *filename)
{
FILE *fp;
int v,entry,n;
uint32_t mac;
int x,y;
char buf[1024];

ap_num=0;
fp=fopen( filename, "r" );
if(fp==NULL) 
	{
	printf( "Could not open beacon database: %s\n",filename );
	exit(0);
	}
entry=-1;
do {
v=fscanf( fp, "%[^\n]\n", buf);
if(v!=-1)
  {
  if(strstr(buf,"#")!=0) continue;
  sscanf( buf,"%X, %d, %d",&mac,&x,&y );
  ap[ap_num].mac=mac;
  ap[ap_num].x=x;
  ap[ap_num].y=y;
  ap_num++;
  }
} while(v!=-1);

printf( "beacon db loaded.\n" );
}


int loc_beacon_centroid(nlist_t *input, beacon_t *result)
{
int i,j;
int sum_x, sum_y,cnt, sum_w;
int w;

cnt=0;
sum_x=0;
sum_y=0;
sum_w=0;

if(input->num<3 && input->num>0)
{
for(i=0; i<ap_num; i++ )
 {
 if(ap[i].mac==input->link_mac[0] )
  {
   result->x=ap[i].x;
   result->y=ap[i].y;
   return 1;
   }
  }
}

for(i=0; i<input->num; i++ )
{
  for(j=0; j<ap_num; j++ )
	{
	if(ap[j].mac==input->link_mac[i] )
		{
		w=input->rssi[i]+50;
		w=pow(w,3)/10;
		sum_x+=(ap[j].x*w);
		sum_y+=(ap[j].y*w);
		sum_w+=w;
		cnt++;
		}
	}
}

if(cnt>0) {
result->x=sum_x/(cnt+sum_w);
result->y=sum_y/(cnt+sum_w);
return 1;
}
return 0;
}


