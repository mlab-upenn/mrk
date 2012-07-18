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
#ifndef _N_LIST_H_
#define _N_LIST_H_

#include <stdint.h>

#define MAX_NEIGHBORS   20 
#define MAX_MOBILE_NODES	20

typedef struct nlist 
{
  uint32_t 	mac;
  uint32_t 	link_mac[MAX_NEIGHBORS];
  int8_t 	rssi[MAX_NEIGHBORS];
  uint8_t 	num;
} nlist_t;

typedef struct  beacon
{
  uint32_t 	mac;
  char		desc[64];
  int32_t	x;
  int32_t	y;
} beacon_t;



void nlist_print(nlist_t *l);
void nlist_sort(nlist_t *l);

#endif
