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
#ifndef _LOC_CENTROID_H_
#define _LOC_CENTROID_H_

#include <stdint.h>
#include <nlist.h>

#define MAX_BEACONS	32 



int ap_num;
beacon_t ap[MAX_BEACONS];


void loc_beacon_print();
void loc_beacon_load(char *file_name);
int loc_beacon_centroid(nlist_t *input, beacon_t *result);

#endif
