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
#ifndef _LOC_ENGINE_H_
#define _LOC_ENGINE_H_

#include <stdint.h>
#include <nlist.h>


void loc_engine_init(char *loc_db_path, char *beacon_path, char *map_path);
void loc_engine_update(nlist_t *l);

#endif
