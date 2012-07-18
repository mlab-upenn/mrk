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
#include <loc_db.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>


void loc_db_print()
{
int i,j;

for(i=0; i<loc_db_elements; i++ )
{
printf( "Zone MAC:  0x%x\n",loc_database[i].zone.mac );
printf( "Zone Desc: %s\n",loc_database[i].zone.desc);
printf( "Zone X: %d\n",loc_database[i].zone.x);
printf( "Zone Y: %d\n",loc_database[i].zone.y);
   for(j=0; j<loc_database[i].beacons.num; j++ )
   {
	printf( "  MAC: 0X%x\tRSSI: %d\n", loc_database[i].beacons.link_mac[j], loc_database[i].beacons.rssi[j] );
   }
printf( "\n" );
}


}

void loc_db_load(char *filename)
{
FILE *fp;
int v,entry,n;
uint32_t mac;
int rssi,x,y;
char buf[1024];

loc_db_elements=0;
fp=fopen( filename, "r" );
if(fp==NULL) 
	{
	printf( "Could not open location database: %s\n",filename );
	exit(0);
	}
entry=-1;
do {
v=fscanf( fp, "%[^\n]\n", buf);
if(v!=-1)
  {
  if(strstr(buf,"#")!=0) continue;
   if(strstr(buf,"LOC_DESC:")!=0 )
	   {
		if(entry>=0) loc_database[entry].beacons.num=n;   
		entry++;
		n=0;
		strtok(buf,":");
		strcpy(loc_database[entry].zone.desc, strtok(NULL,":" ));
		//printf( "  Location: %s\n",loc_database[entry].loc_desc );
	   }
   if(strstr(buf,"LINK:")!=0 )
	   {
		strtok(buf,":");
		sscanf( strtok(NULL,": "), "%X",&mac);
		sscanf( strtok(NULL," "), "%d",&rssi);
		loc_database[entry].beacons.link_mac[n]=mac;
		loc_database[entry].beacons.rssi[n]=rssi;
		if(n<MAX_NEIGHBORS) n++;
	   }
   if(strstr(buf,"COORD:")!=0 )
	   {
		strtok(buf,":");
		sscanf( strtok(NULL,": "), "%d",&x);
		sscanf( strtok(NULL," "), "%d",&y);
		loc_database[entry].zone.x=x;
		loc_database[entry].zone.y=y;
	   }


  }
} while(v!=-1);

if(entry>=0) loc_database[entry].beacons.num=n;   
loc_db_elements=entry+1;
printf( "loc db loaded.\n" );
}



beacon_t* loc_db_find_nn(nlist_t *input, int k_val)
{

int i,j,k, min_d, min_i,d,v,cnt;
int last_rssi;
min_d=0xffff;
min_i=-1;

if(input->num<k_val) return NULL;

for(i=0; i<loc_db_elements; i++ )
{
d=0;
cnt=0;
for(j=0; j<loc_database[i].beacons.num; j++ )
	{
	int last_rssi,kc,trssi,p;

		for(k=0; k<k_val; k++ )
	 	if( loc_database[i].beacons.link_mac[j] == input->link_mac[k] )
		    {
			v=loc_database[i].beacons.rssi[j]-input->rssi[k];
			if(v<0) v*=-1;
			d+=v;
			cnt++;
		    }

	}

// Make sure it is smaller and has enough data points (they should all match)
if(d<min_d && cnt>=k_val) {
	min_i=i;
	min_d=d;
	}
}
if(min_i==-1) return NULL;
return &loc_database[min_i].zone;
}


