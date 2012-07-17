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
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

void nlist_print(nlist_t *l)
{
int i;

printf( "Mobile MAC: %x\n",l->mac);
for(i=0; i<l->num; i++ )
	{
	printf( "LINK: 0x%X %d\n",  l->link_mac[i], l->rssi[i]);
	}
printf( "\n" );

}
// Crappy bubble sort, but the data sets are small...
void nlist_sort(nlist_t *l)
{
int i,j;
uint32_t tmp_mac;
uint32_t tmp_rssi;

if(l->num<2) return;
for(i=0; i<l->num-1; i++ )
	for(j=0; j<l->num-1; j++ )
	{
	if(l->rssi[j]<l->rssi[j+1])
		{
		tmp_mac=l->link_mac[j];
		tmp_rssi=l->rssi[j];
		l->rssi[j]=l->rssi[j+1];
		l->link_mac[j]=l->link_mac[j+1];
		l->rssi[j+1]=tmp_rssi;
		l->link_mac[j+1]=tmp_mac;
		}
	}

}



