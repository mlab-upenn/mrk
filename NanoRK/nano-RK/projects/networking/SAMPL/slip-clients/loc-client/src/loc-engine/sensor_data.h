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
#ifndef _SENSOR_DATA_H_
#define _SENSOR_DATA_H_

#include <stdint.h>

#define MAX_SENSOR_NODES	10

typedef struct sensor_data 
{
  char time_str[100];
  uint8_t data[128];
  uint8_t data_size;
  uint32_t mac;
} sensor_data_t;

int sensor_data_elements;

sensor_data_t sensor_cache[MAX_SENSOR_NODES];

void sensor_data_init();
void sensor_data_print();
int8_t sensor_data_get(uint32_t mac, char *str);
int8_t sensor_data_add(uint32_t mac, char *str);

#endif
