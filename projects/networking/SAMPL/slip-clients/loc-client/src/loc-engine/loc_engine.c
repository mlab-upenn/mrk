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
#include <nlist.h>
#include <loc_engine.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <map.h>
#include <loc_centroid.h>

static beacon_t mobile_node[MAX_MOBILE_NODES];

static int node_cnt;

void loc_engine_init(char *loc_db_path, char *beacon_path, char *map_path)
{
int i;
  printf( "Location engine starting up\n" );
  loc_db_load(loc_db_path);
  loc_beacon_load(beacon_path);
  map_init(map_path);
  for(i=0; i<MAX_MOBILE_NODES; i++ ) mobile_node[i].mac=0;
  //loc_beacon_print();
  //loc_db_print();
  node_cnt=0;
  map_render(&ap,ap_num, NULL, 0);
}

void loc_engine_update(nlist_t *l)
{
beacon_t *zone;
beacon_t centroid;
int v,i,got_centroid,got_sig;

// Just a sensor packet
if(l->num==0)
{
  map_render(&ap,ap_num, &mobile_node, node_cnt);
  return;
}
  got_centroid=0;
  got_sig=0;
  node_cnt=0;


  nlist_sort(l);
  nlist_print(l);
  v=loc_beacon_centroid(l,&centroid);
  if(v==1) { got_centroid=1; printf( "Centroid: %d, %d\n",centroid.x, centroid.y ); }
   else printf( "Centroid could not find any APs\n" );

  l->num=3;
  zone=loc_db_find_nn(l,3);
  if(zone!=NULL ) {
	printf( "Signature Location: %s ", zone->desc );
	printf( "Coordinates: %d, %d\n", zone->x, zone->y);
	got_sig=1;
	}
  else printf( "Signature Location unknown: Need more data\n" );

printf( "Setting mobile node data\n" );

for(i=0; i<MAX_MOBILE_NODES; i+=2 )
{
if(mobile_node[i].mac==l->mac || mobile_node[i].mac==0)
{
  mobile_node[i].mac=l->mac;
  mobile_node[i].x=centroid.x;
  mobile_node[i].y=centroid.y;
  if(got_centroid==1) mobile_node[i].desc[0]='C';
  else mobile_node[i].desc[0]='X';
  mobile_node[i+1].mac=l->mac;
  if(got_sig==1)
  {	
  mobile_node[i+1].x=zone->x;
  mobile_node[i+1].y=zone->y;
  }
  if(got_sig==1) mobile_node[i+1].desc[0]='S';
  else  mobile_node[i+1].desc[0]='X';
  node_cnt=i+2;
  break;
}
}

  printf( "Calling map render\n" );

  map_render(&ap,ap_num, &mobile_node, node_cnt);
}



