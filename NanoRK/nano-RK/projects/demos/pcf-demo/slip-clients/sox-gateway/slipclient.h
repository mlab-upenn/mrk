/******************************************************************************
 *  Nano-RK, a real-time operating system for sensor networks.
 *  SLIPstream Client
 *  Copyright (C) 2011, Real-Time and Multimedia Lab, Carnegie Mellon University
 *  All rights reserved.
 *
 *  This is the Open Source Version of Nano-RK included as part of a Dual
 *  Licensing Model. If you are unsure which license to use please refer to:
 *  http://www.nanork.org/nano-RK/wiki/Licensing
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
 *
 *  Contributing Authors (specific to this file):
 *  Patrick Lazik
 *******************************************************************************/

#ifndef SLIPCLIENT_H_
#define SLIPCLIENT_H_

/* SOX related inclusions */
#ifdef USE_SOX
#include <strophe.h>
#include <sox_handlers.h>
#include <common.h>
#include <sox.h>
#endif

/* SAGA related inclusions */
#ifdef USE_SAGA
#include <ffdb.h>
#endif

#define MAX_TRANSDUCERS 12 // Maximum number of transducers per node
#define MAX_NODES 100 // Maximum number of nodes
#define MAX_TYPES 10 // Maximum number of different types of nodes
#define MAX_STRING_SHORT 8
#define MAX_STRING_LONG 128
#define MAX_STRING_VLONG 512

#define NONBLOCKING  0
#define BLOCKING     1
#define SLIP_KEEP_ALIVE_PERIOD 100000000

#define SAGA_DELAY 3000000
#define REFRESH 100	// Refresh rate in µs

#define POW_THRESHOLD_POWER 80000
#define POW_LOW_TO_HIGH_FACTOR 21

#define SAGA_MULTIPLIER 1000 // Factor by which the typed float value is multiplied to convert to int so that SAGA can read it

#define MAX_USER_INPUT_CHARS 128

#define SAGA_PKT_POW_INDEX_TOTALSECS 0
#define SAGA_PKT_POW_INDEX_FREQ 1
#define SAGA_PKT_POW_INDEX_VOLTAGE 2
#define SAGA_PKT_POW_INDEX_CURRENT 3
#define SAGA_PKT_POW_INDEX_POWER 4
#define SAGA_PKT_POW_INDEX_ENERGY 5
#define SAGA_PKT_POW_INDEX_STATE 6
#define SAGA_PKT_POW_INDEX_CURRENT1 7
#define SAGA_PKT_POW_INDEX_POWER1 8
#define SAGA_PKT_POW_INDEX_ENERGY1 9
#define SAGA_PKT_POW_INDEX_STATE1 10

#define SAGA_PKT_ENV_INDEX_BAT 0
#define SAGA_PKT_ENV_INDEX_LIGHT 1
#define SAGA_PKT_ENV_INDEX_TEMP 2
#define SAGA_PKT_ENV_INDEX_ACCX 3
#define SAGA_PKT_ENV_INDEX_ACCY 4
#define SAGA_PKT_ENV_INDEX_ACCZ 5
#define SAGA_PKT_ENV_INDEX_AUDIO 6
#define SAGA_PKT_ENV_INDEX_HUMIDITY 7
#define SAGA_PKT_ENV_INDEX_DIGITALTEMP 8
#define SAGA_PKT_ENV_INDEX_PRESSURE 9
#define SAGA_PKT_ENV_INDEX_MOTION 10
#define SAGA_PKT_ENV_INDEX_GPIOSTATE 11

/* Structure definitions */
typedef struct transducer {
	char id[MAX_STRING_LONG];
	uint64_t raw_val;
	float val;
	uint8_t type;
	uint8_t raw_val_len;
	float r_to_tval_factor;
	float cal_factor;
	uint8_t invert;
} transducer;

typedef struct node {
	uint8_t type;
	char event[MAX_STRING_LONG];
	char id[MAX_STRING_VLONG];
	uint32_t mac;
	uint8_t num_transducers;
	struct transducer transducers[MAX_TRANSDUCERS];
	uint8_t data_fresh;
#ifdef USE_SOX
	sox_stanza_t *item;
#endif
#ifdef USE_SAGA
	FF_ENV_PKT * ff_env_pkt;
	FF_POW_PKT * ff_pow_pkt;
#endif
} node;

//typedef void (*xml_call)(void *data, const char *element_name, const char **attr);

#endif /* SLIPCLIENT_H_ */
