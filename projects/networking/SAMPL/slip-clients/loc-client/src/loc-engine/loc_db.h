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
#ifndef _LOC_DB_H_
#define _LOC_DB_H_

#include <stdint.h>
#include <nlist.h>

#define MAX_NEIGHBORS   20 
#define MAX_LOCATIONS   20 


typedef struct loc
{
  beacon_t zone;
  nlist_t beacons;
} loc_t;

int loc_db_elements;

loc_t loc_database[MAX_LOCATIONS];

void loc_db_print();
void loc_db_load(char *file_name);
beacon_t* loc_db_find_nn(nlist_t *input, int k_val);

#endif
