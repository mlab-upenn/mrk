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

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <slipstream.h>
#include <expat.h>
#include <pthread.h>
#include <math.h>
#include <pkt.h>
#include <slipclient.h>

/* Actuating nodes */
#define NUM_ACT_NODES 10 // Number of nodes that can actuate
char act_nodes[NUM_ACT_NODES][50] = { "d4ee66c9184c07880d507103ec61c50c_data",
		"d4ee66c9184c07880d507103ec61c50c_data",
		"d4ee66c9184c07880d507103ec61c50c_data" };
int act_nodes_macs[NUM_ACT_NODES] = { 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };
int act_index = -1;

/* Function Prototypes */
static void XMLCALL XMLstart_read_conf_default(void *, const char *,
		const char **);
static void XMLCALL XMLstart_read_conf_optional(void *, const char *,
		const char **);
static void XMLCALL endElement(void *, const char *);
void print_usage(char *);
uint8_t read_conf(char *, void(*f)(void *, const char *, const char **));
void *slip_watcher(void *ud);
uint8_t write_vals(PKT_T *, uint8_t *);
void write_csv(uint8_t *);
void *slip_keep_alive(void *);
int outlet_actuate(uint32_t mac, uint8_t socket, uint8_t state);
int kbhit();
void *user_input_thread(void *ud);

#ifdef USE_SOX
void conn_handler(sox_conn_t * const, const sox_conn_event_t,
		sox_userdata_t * const);
uint8_t write_vals_sox(uint8_t *);
int pubsub_handler(sox_conn_t *conn, sox_stanza_t *stanza,
		sox_userdata_t *userdata);
static void XMLCALL startElement(void *data, const char *element_name,
		const char **attr);
void write_vals_csv(uint8_t *);
#endif
#ifdef USE_SAGA
void *write_saga(void *);
#endif

/* Global Variables */
node proto_nodes[MAX_TYPES]; // prototype node structures
node nodes[MAX_NODES];
uint8_t proto_nodes_bound = 0;
uint8_t nodes_bound = 0;
uint8_t transducer_i;
uint8_t node_i = 0;
uint8_t verbose = 0;
uint8_t use_sox = 0;
#ifdef USE_SOX
sox_conn_t * global_conn = NULL;
#endif

time_t raw_time;
char curr_day[3] = "0";
char prev_day[3] = "0";
FILE *csv_fp;
char name[MAX_STRING_LONG] = "";
int trans_found = 0;

node temp_node;
transducer temp_transducer;

char *username = NULL;
char *csv = NULL;
char *def = NULL;
char *opt = NULL;

pthread_t slip_watcher_thread;

/* Function to parse default configuration data */
static void XMLCALL XMLstart_read_conf_default(void *data,
		const char *element_name, const char **attr) {

	uint16_t i;

	if (strcmp(element_name, "payload") == 0) {
		proto_nodes_bound++;
		transducer_i = 0;
		for (i = 0; attr[i]; i += 2) {
			const char* attr_name = attr[i];
			if (strcmp(attr_name, "type") == 0) {
				proto_nodes[proto_nodes_bound - 1].type = atoi(attr[i + 1]);
			}
		}
	}

	else if (strcmp(element_name, "transducer") == 0) {
		proto_nodes[proto_nodes_bound - 1].num_transducers++;
		proto_nodes[proto_nodes_bound - 1].transducers[transducer_i].cal_factor =
		1;
		proto_nodes[proto_nodes_bound - 1].transducers[transducer_i].r_to_tval_factor =
		1;
		if (proto_nodes[proto_nodes_bound - 1].num_transducers > MAX_TRANSDUCERS) {
			fprintf(
					stderr,
					"Error: Too many transducers specified in default configuration file for node of type %u, please increase MAX_TRANSDUCERS in slipclient.h\n",
					proto_nodes[proto_nodes_bound - 1].type);
			exit(1);
		}
		for (i = 0; attr[i]; i += 2) {
			const char* attr_name = attr[i];
			if (strcmp(attr_name, "id") == 0) {
				strcpy(
						proto_nodes[proto_nodes_bound - 1].transducers[transducer_i].id,
						attr[i + 1]);
			} else if (strcmp(attr_name, "rVLen") == 0) {
				proto_nodes[proto_nodes_bound - 1].transducers[transducer_i].raw_val_len =
				atoi(attr[i + 1]);
			} else if (strcmp(attr_name, "type") == 0) {
				proto_nodes[proto_nodes_bound - 1].transducers[transducer_i].type =
				atoi(attr[i + 1]);
			} else if (strcmp(attr_name, "invert") == 0) {
				if (atoi(attr[i + 1]) != 0 && atoi(attr[i + 1]) != 1) {
					fprintf(
							stderr,
							"Error: Invert parameter of node type %u and transducer id %s, specified in default configuration file must be either \"0\" or \"1\".\n",
							proto_nodes[proto_nodes_bound - 1].type,
							proto_nodes[proto_nodes_bound - 1].transducers[transducer_i].id);
					exit(1);
				}
				proto_nodes[proto_nodes_bound - 1].transducers[transducer_i].invert =
				atoi(attr[i + 1]);
			} else if (strcmp(attr_name, "rToTFactor") == 0) {
				sscanf(
						attr[i + 1],
						"%f",
						&proto_nodes[proto_nodes_bound - 1].transducers[transducer_i].r_to_tval_factor);
			} else if (strcmp(attr_name, "calFactor") == 0) {
				sscanf(
						attr[i + 1],
						"%f",
						&proto_nodes[proto_nodes_bound - 1].transducers[transducer_i].cal_factor);
			}
		}
		transducer_i++;
	}
}

/* Function to parse optional configuration data */
static void XMLCALL XMLstart_read_conf_optional(void *data,
		const char *element_name, const char **attr) {

	uint16_t i;
	uint8_t j;
	int8_t err = 2; // Assume there will be an error

	if (strcmp(element_name, "node") == 0) {
		for (i = 0; attr[i]; i += 2) {
			const char* attr_name = attr[i];
			if (strcmp(attr_name, "mac") == 0) {
			  if(strstr(attr[i+1],"0x" )!=NULL)
					sscanf(attr[i+1],"%x",&temp_node.mac );
				else
					temp_node.mac = atoi(attr[i + 1]);
				
				err--;
			} else if (strcmp(attr_name, "event") == 0) {
				strcpy(temp_node.event, attr[i + 1]);
			} else if (strcmp(attr_name, "type") == 0) {
				temp_node.type = atoi(attr[i + 1]);
				for (j = 0; j < MAX_TYPES; j++) {
					if (proto_nodes[j].type == temp_node.type) {
						nodes[node_i] = proto_nodes[j];
						nodes_bound++;
						node_i++;
						err--;
						break;
					}
				}
				if (j == MAX_TYPES) {
					fprintf(
							stderr,
							"Error: Node type %u specified in optional configuration file, but not in default configuration.\n",
							temp_node.type);
					exit(1);
				}
			}
		}
		if (err == 0) { // err == 0 if mac and type attributes have been read
			strcpy(nodes[node_i - 1].event, temp_node.event);
			nodes[node_i - 1].mac = temp_node.mac;
		} else {
			fprintf(
					stderr,
					"Error: Either no MAC and/or type specified for node in optional configuration file.\n");
			exit(1);
		}
	}

	else if (strcmp(element_name, "transducer") == 0) {
		temp_transducer.r_to_tval_factor = 0;
		temp_transducer.cal_factor = 0;
		temp_transducer.invert = 2;
		for (i = 0; attr[i]; i += 2) {
			const char* attr_name = attr[i];
			if (strcmp(attr_name, "regID") == 0) {
				strcpy(name, attr[i + 1]);
			}
			else if (strcmp(attr_name, "id") == 0) {
				for (transducer_i = 0; transducer_i < MAX_TRANSDUCERS;
						transducer_i++) {
					if (strcmp(attr[i + 1], nodes[node_i - 1].transducers[transducer_i].id) == 0){
						trans_found = 1;
						break;
					}
				}
				if (transducer_i == MAX_TRANSDUCERS) {
					fprintf(
							stderr,
							"Error: Transducer with ID %s of node with MAC %u is not specified in the default packet of type %u.\n",
							attr[i + 1], nodes[node_i - 1].mac,
							nodes[node_i - 1].type);
					exit(1);
				}
			} else if (strcmp(attr_name, "rToTFactor") == 0) {
				sscanf(attr[i + 1], "%f",
						&temp_node.transducers[transducer_i].r_to_tval_factor);
			} else if (strcmp(attr_name, "invert") == 0) {
				if (atoi(attr[i + 1]) != 0 && atoi(attr[i + 1]) != 1) {
					fprintf(
							stderr,
							"Error: Invert parameter of node type with MAC %u, specified in optional configuration file must be either \"0\" or \"1\".\n",
							nodes[node_i - 1].type);
					exit(1);
				}
				temp_node.transducers[transducer_i].invert = atoi(attr[i + 1]);
			} else if (strcmp(attr_name, "calFactor") == 0) {
				sscanf(attr[i + 1], "%f",
						&temp_node.transducers[transducer_i].cal_factor);
			}
		}
		if (temp_transducer.r_to_tval_factor != 0)
			nodes[node_i - 1].transducers[transducer_i].r_to_tval_factor =
					temp_transducer.r_to_tval_factor;
		if (temp_transducer.cal_factor != 0)
			nodes[node_i - 1].transducers[transducer_i].cal_factor =
					temp_transducer.cal_factor;
		if (temp_transducer.invert != 2)
			nodes[node_i - 1].transducers[transducer_i].invert =
					temp_transducer.invert;
		if(strcmp(name, "") && trans_found == 1){
			strcpy(nodes[node_i - 1].transducers[transducer_i].id, name);
			trans_found = 0;
			strcpy(name, "");
		}
	}

}

// XMLParser function called whenever an end of element is encountered
static void XMLCALL endElement(void *data, const char *element_name) {
}

/* Function to read configuration files */
uint8_t read_conf(char* fileloc,
		void(*parser)(void *, const char *, const char **)) {
	FILE * pFile;
	char Buff[512];
	XML_Parser p = XML_ParserCreate(NULL);

	if (!p) {
		fprintf(stderr, "Couldn't allocate memory for parser\n");
		return -1;
	}

	XML_SetElementHandler(p, parser, endElement);

	/* Notice that the default handler is not set at this point */
	pFile = fopen(fileloc, "r");
	if (pFile == NULL) {
		fprintf(stderr, "Error opening file %s", fileloc);
		return -1;
	}

	for (;;) {
		int done;
		int len;

		fgets(Buff, sizeof(Buff), pFile);
		len = strlen(Buff);
		if (ferror(pFile)) {
			fprintf(stderr, "Read error\n");
			return -1;
		}
		done = feof(pFile);
		if (!XML_Parse(p, Buff, len, done)) {
			fprintf(stderr, "Parse error at line %d:\n%s\n",
					(int) XML_GetCurrentLineNumber(p),
					XML_ErrorString(XML_GetErrorCode(p)));
			return -1;
		}

		if (done)
			break;
	}

	if (opt != NULL && parser != XMLstart_read_conf_optional)
		return read_conf(opt, XMLstart_read_conf_optional);

	return 0;
}

#ifdef USE_SOX
static void XMLCALL startElement(void *data, const char *element_name,
		const char **attr) {
	int i, j;
	char node[50] = "";

	/*if (strcmp(element_name, "on") == 0)
	 outlet_actuate(3, 0, 1);
	 else if (strcmp(element_name, "off") == 0)
	 outlet_actuate(3, 0, 0);*/

	if (strcmp(element_name, "publish") == 0) {
		for (i = 0; attr[i]; i += 2) {
			const char* attr_name = attr[i];
			if (strcmp(attr_name, "id") == 0) {
				for (j = 0; j < NUM_ACT_NODES; j++) {
					if(strcmp(attr_name, act_nodes[j]) == 0)
					act_index = j;
				}
			}
		}
	}
	else if (strcmp(element_name, "transducerValue") == 0 && act_index >= 0) {
		for (i = 0; attr[i]; i += 2) {
			const char* attr_name = attr[i];
			if (strcmp(attr_name, "rawValue") == 0) {
				if (atoi(attr[i + 1]) == 0)
				outlet_actuate(act_nodes_macs[act_index], 0, 0);
				else
				outlet_actuate(act_nodes_macs[act_index], 0, 1);
				act_index = -1;
			}
		}
	}
}

void conn_handler(sox_conn_t * const conn, const sox_conn_event_t status,
		sox_userdata_t * const userdata) {

	xmpp_stanza_t *pres;

	fprintf(stdout, "Connected to XMPP server!\n");
	if (status == SOX_CONN_CONNECT) {
		global_conn = conn;
		use_sox = 1;
	}

//Send initial <presence/> so that we appear online to contacts
	pres = xmpp_stanza_new(conn->xmpp_conn->ctx);
	xmpp_stanza_set_name(pres, "presence");
	xmpp_send(conn->xmpp_conn, pres);
	xmpp_stanza_release(pres);

	sox_handler_add(conn, pubsub_handler,
			"http://jabber.org/protocol/pubsub#event", NULL, NULL, userdata);
}

int pubsub_handler(sox_conn_t *conn, sox_stanza_t *stanza,
		sox_userdata_t *userdata) {

	XML_Parser p;
	char *buf;
	size_t buflen;
	if (stanza == NULL)
		return -1;

	xmpp_stanza_to_text(stanza->xmpp_stanza, &buf, &buflen);

//Creates an instance of the XML Parser to parse the event packet
	p = XML_ParserCreate(NULL);
	if (!p) {
		fprintf(stderr, "Couldn't allocate memory for XML Parser\n");
		return -1;
	}

//Sets the handlers to call when parsing the start and end of an XML element
	XML_SetElementHandler(p, startElement, endElement);

	if (XML_Parse(p, buf, strlen(buf), 1) == XML_STATUS_ERROR) {
		fprintf(stderr, "Parse Error at line %u:\n%s\n",
				XML_GetCurrentLineNumber(p),
				XML_ErrorString(XML_GetErrorCode(p)));
		return -1;
	}

	XML_ParserFree(p);
	fprintf(stdout, "\n");

	return 1;
}
#endif

/* Function to process and write incoming data to node/transducer structs */
uint8_t write_vals(PKT_T * pkt, uint8_t *nodep) {
	uint8_t i, j, k, offset = 0;

// Sanity checks on received packet
	if (pkt->payload_len < MIN_PAYLOAD || pkt->payload_len > MAX_PAYLOAD) {
		fprintf(stdout, "Error: Payload not within expected length\n");
		return -1;
	}

// Find the node in the config file list if it exists
	for (*nodep = 0; *nodep < nodes_bound; (*nodep)++) {
		if (nodes[*nodep].mac == pkt->src_mac)
			break;
	}
// If not, create a new node
	if (*nodep == MAX_NODES) {
		fprintf(
				stderr,
				"Error: Maximum node count reached, ignoring packet from node %u\n",
				pkt->src_mac);
		return -1;
	} else {
		if (*nodep == nodes_bound) {
			for (i = 0; i < proto_nodes_bound; i++) {
				if (proto_nodes[i].type == pkt->payload[PAYLOAD_TYPE_INDEX]) {
					nodes[*nodep] = proto_nodes[i];
					nodes_bound++;
					break;
				}
			}
			if (i == proto_nodes_bound) {
				fprintf(
						stderr,
						"Error: Packet type %u received from node 0x%x is not listed in the default configuration file\n",
						pkt->payload[PKT_TYPE_INDEX], pkt->src_mac);
				return -1;
			}
		}
	}

// Write transducer values from packet
	offset = PAYLOAD_VALUE_START_INDEX;
	nodes[*nodep].data_fresh = 1;
	nodes[*nodep].mac = pkt->src_mac;
	for (i = 0; i < nodes[*nodep].num_transducers; i++) {
		k = nodes[*nodep].transducers[i].raw_val_len - 1;
		nodes[*nodep].transducers[i].raw_val = 0;
		for (j = 0; j < nodes[*nodep].transducers[i].raw_val_len; j++) {
			nodes[*nodep].transducers[i].raw_val |=
					(uint64_t) pkt->payload[offset + j] << (k * 8);
			k--;
		}

		if (nodes[*nodep].transducers[i].invert == 1)
			nodes[*nodep].transducers[i].val = (float) (((float) pow(2,
					(nodes[*nodep].transducers[i].raw_val_len - 1))
					/ (nodes[*nodep].transducers[i].raw_val + 1)) - 1)
					* nodes[*nodep].transducers[i].cal_factor
					* nodes[*nodep].transducers[i].r_to_tval_factor;

		else
			nodes[*nodep].transducers[i].val =
					nodes[*nodep].transducers[i].raw_val
							* nodes[*nodep].transducers[i].cal_factor
							* nodes[*nodep].transducers[i].r_to_tval_factor;

		offset += j;
	}
	return 0;
}

#ifdef USE_SOX
/* Function to write incoming data to SOX stanzas */
uint8_t write_vals_sox(uint8_t *nodep) {
	char time_str[30];
	char raw_val[MAX_STRING_LONG];
	char typed_val[MAX_STRING_LONG];
	uint8_t i;

	if (nodes[*nodep].event == NULL)
		return -1;

// Construct new item stanza
//if (nodes[*nodep].item != NULL)
//xmpp_stanza_release(nodes[*nodep].item->xmpp_stanza);

	nodes[*nodep].item = sox_pubsub_item_data_new(global_conn);
// Create id for iq stanza consisting of "pub_" and node name
// String length is length of server address + length of "pub_" + terminating character

  strcpy(nodes[*nodep].id, "pub_");
	strcat(nodes[*nodep].id, nodes[*nodep].event);
	sox_timestamp_create(time_str);

	for (i = 0; i < nodes[*nodep].num_transducers; i++) {
		sprintf(raw_val, "%lu", nodes[*nodep].transducers[i].raw_val);
		sprintf(typed_val, "%f", nodes[*nodep].transducers[i].val);
		if (sox_item_transducer_value_add(nodes[*nodep].item, NULL,
				nodes[*nodep].transducers[i].id, typed_val, raw_val, time_str)
				< 0) {
			fprintf(
					stderr,
					"ERROR: TransducerValue could not be added to transducer with ID %s.\n",
					nodes[*nodep].transducers[i].id);
			return -1;
		}
	}
	return 0;
}
#endif

#ifdef USE_SAGA
/* Function to write incoming data to SAGA packets */
uint8_t write_vals_saga(uint8_t *nodep) {
	uint8_t i;
	char tempstr[MAX_STRING_LONG];

	if (nodes[*nodep].type == TYPE_PKT_POWER) {
		if (nodes[*nodep].ff_pow_pkt == NULL) {
			nodes[*nodep].ff_pow_pkt = (FF_POW_PKT *) malloc(
					sizeof(FF_POW_PKT));
			sprintf(tempstr, "%u", nodes[*nodep].mac);
			nodes[*nodep].ff_pow_pkt->id = (char *) malloc(
					(strlen(tempstr) + 1) * sizeof(char));
			strcpy(nodes[*nodep].ff_pow_pkt->id, tempstr);
		}
		nodes[*nodep].ff_pow_pkt->time = (unsigned int) time(NULL);
		nodes[*nodep].ff_pow_pkt->rms_voltage =
		(int) (nodes[*nodep].transducers[SAGA_PKT_POW_INDEX_VOLTAGE].val
				* SAGA_MULTIPLIER);
		nodes[*nodep].ff_pow_pkt->state =
		(int) (nodes[*nodep].transducers[SAGA_PKT_POW_INDEX_STATE].raw_val
				* SAGA_MULTIPLIER);

		// If we are above the threshold power, use the non-amplified values
		if (nodes[*nodep].transducers[SAGA_PKT_POW_INDEX_POWER1].raw_val
				> POW_THRESHOLD_POWER) {
			nodes[*nodep].ff_pow_pkt->rms_current =
			(int) (nodes[*nodep].transducers[SAGA_PKT_POW_INDEX_CURRENT].val
					* POW_LOW_TO_HIGH_FACTOR * SAGA_MULTIPLIER);
			nodes[*nodep].ff_pow_pkt->true_power =
			(int) (nodes[*nodep].transducers[SAGA_PKT_POW_INDEX_POWER].val
					* POW_LOW_TO_HIGH_FACTOR * SAGA_MULTIPLIER);
			nodes[*nodep].ff_pow_pkt->energy =
			(int) (nodes[*nodep].transducers[SAGA_PKT_POW_INDEX_ENERGY].val
					* POW_LOW_TO_HIGH_FACTOR * SAGA_MULTIPLIER);
		} else {
			nodes[*nodep].ff_pow_pkt->rms_current =
			(int) (nodes[*nodep].transducers[SAGA_PKT_POW_INDEX_CURRENT1].val
					* SAGA_MULTIPLIER);
			nodes[*nodep].ff_pow_pkt->true_power =
			(int) (nodes[*nodep].transducers[SAGA_PKT_POW_INDEX_POWER1].val
					* SAGA_MULTIPLIER);
			nodes[*nodep].ff_pow_pkt->energy =
			(int) (nodes[*nodep].transducers[SAGA_PKT_POW_INDEX_ENERGY1].val
					* SAGA_MULTIPLIER);
		}
		return 0;
	} else if (nodes[*nodep].type == TYPE_PKT_ENVIRONMENTAL) {
		if (nodes[*nodep].ff_env_pkt == NULL) {
			nodes[*nodep].ff_env_pkt = (FF_ENV_PKT *) malloc(
					sizeof(FF_ENV_PKT));
			sprintf(tempstr, "%u", nodes[*nodep].mac);
			nodes[*nodep].ff_env_pkt->id = (char *) malloc(
					(strlen(tempstr) + 1) * sizeof(char));
			strcpy(nodes[*nodep].ff_env_pkt->id, tempstr);
		}
		nodes[*nodep].ff_env_pkt->time = (unsigned int) time(NULL);
		nodes[*nodep].ff_env_pkt->accl =
		(int) (nodes[*nodep].transducers[SAGA_PKT_ENV_INDEX_ACCX].val
				* SAGA_MULTIPLIER);
		nodes[*nodep].ff_env_pkt->arb1 =
		(int) (nodes[*nodep].transducers[SAGA_PKT_ENV_INDEX_HUMIDITY].val
				* SAGA_MULTIPLIER);
		nodes[*nodep].ff_env_pkt->arb2 =
		(int) (nodes[*nodep].transducers[SAGA_PKT_ENV_INDEX_ACCZ].val
				* SAGA_MULTIPLIER);
		nodes[*nodep].ff_env_pkt->audio =
		(int) (nodes[*nodep].transducers[SAGA_PKT_ENV_INDEX_AUDIO].val
				* SAGA_MULTIPLIER);
		nodes[*nodep].ff_env_pkt->light =
		(int) (nodes[*nodep].transducers[SAGA_PKT_ENV_INDEX_LIGHT].val
				* SAGA_MULTIPLIER);
		nodes[*nodep].ff_env_pkt->temp =
		(int) (nodes[*nodep].transducers[SAGA_PKT_ENV_INDEX_DIGITALTEMP].val
				* SAGA_MULTIPLIER);
		nodes[*nodep].ff_env_pkt->voltage =
		(int) (nodes[*nodep].transducers[SAGA_PKT_ENV_INDEX_PRESSURE].val
				* SAGA_MULTIPLIER);
		return 0;
	}
	return -1;
}
#endif

/* Thread to write CSV data */
void write_vals_csv(uint8_t *nodep) {

	char tempstr[MAX_STRING_VLONG];
	uint8_t i;

	time(&raw_time);
	strftime(curr_day, 100, "%d", localtime(&raw_time));

	if (atoi(curr_day) != atoi(prev_day)) {
		strcpy(prev_day, curr_day);
		strcpy(tempstr, csv);
		strftime(tempstr + strlen(tempstr), 100, "_%Y_%m_%d.out",
				localtime(&raw_time));
		if (csv_fp != NULL)
			fclose(csv_fp);
		csv_fp = fopen(tempstr, "a");
		if (csv_fp == NULL) {
			fprintf(stderr, "Error opening or creating file %s\n", tempstr);
			exit(1);
		}
	}

	//strftime(tempstr, 100, "%X", localtime(&raw_time));
	//fprintf(csv_fp, "%s,%u", tempstr, nodes[*nodep].mac);
	fprintf(csv_fp, "%lu,%u,%u", time(NULL), nodes[*nodep].mac, nodes[*nodep].type);
	for (i = 0; i < nodes[*nodep].num_transducers; i++)
		fprintf(csv_fp, ",%u", nodes[*nodep].transducers[i].raw_val);
	fprintf(csv_fp, "\n");
	fflush(csv_fp);
}

#ifdef USE_SAGA
/* Thread to write most current data to SAGA database */
void *write_saga(void *ud) {
	uint8_t nodep, cnt = 0;

	while (1) {
		usleep(SAGA_DELAY);
		for (nodep = 0; nodep < nodes_bound; nodep++) {
			if (nodes[nodep].data_fresh) {
				if (nodes[nodep].ff_env_pkt != NULL) {
					write_ff_env(*nodes[nodep].ff_env_pkt);
					cnt++;
				} else if (nodes[nodep].ff_pow_pkt != NULL) {
					write_power(*nodes[nodep].ff_pow_pkt);
					cnt++;
				}
				nodes[nodep].data_fresh = 0;
			}
		}

		printf("wrote %u\n", cnt);
		cnt = 0;
	}
}
#endif

/* Thread to send periodic messages to the SLIPstream server to keep the connection alive */
void *slip_keep_alive(void *ud) {
	int size;
	PKT_T my_pkt;
	char buffer[128];

	/* FIXME: This is a hack to keep SLIP alive while not crashing the master node with an invalid packet */
	my_pkt.src_mac = 0x00000000;
	my_pkt.dst_mac = 0x00000000;
	my_pkt.type = APP;
	my_pkt.payload_len = 0;
	size = pkt_to_buf(&my_pkt, buffer);
	while (1) {
		slipstream_send(buffer, size);
		usleep(SLIP_KEEP_ALIVE_PERIOD);
	}
}

int kbhit() {
	struct timeval tv;
	fd_set fds;
	tv.tv_sec = 0;
	tv.tv_usec = 0;
	FD_ZERO(&fds);
	FD_SET(STDIN_FILENO, &fds);
//STDIN_FILENO is 0
	select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);
	return FD_ISSET(STDIN_FILENO, &fds);
}

/* Thread to check for user input in the terminal */
void *user_input_thread(void *ud) {
	int i, char_count;
	char c, buffer[MAX_USER_INPUT_CHARS];
	char * curr_ch;
	int mac = -1, state = -1;

	while (1) {
		usleep(100000);
		i = kbhit();
		if (i != 0) {
			c = fgetc(stdin);
			char_count = 0;
			while ((c != '\n') && (char_count < MAX_USER_INPUT_CHARS)) {
				buffer[char_count++] = c;
				c = fgetc(stdin);
			}
			buffer[char_count] = 0x00; /* null terminate buffer */

			curr_ch = strtok(buffer, " -/");
			if (!strcmp(curr_ch, "a")) {
				curr_ch = strtok(NULL, " /");
				if (curr_ch != NULL) {
					mac = atoi(curr_ch);
					curr_ch = strtok(NULL, "/");
					if (curr_ch != NULL && mac > 0)
						state = atoi(curr_ch);
				}
			}
			if (state == 0 || state == 1) {
				fprintf(stdout,
						"Actuating plug meter with MAC %u to state %u\n", mac,
						state);
				outlet_actuate(mac, 0, state);
				state = -1;
				mac = -1;
			} else
				fprintf(stderr, "Input error! Usage: -a MAC/state\n");

		}
	}
}

/* Thread to receive and process SLIPstream data */
void *slip_watcher(void *ud) {
	PKT_T pkt;
	char buffer[128];
	int8_t v;
	uint8_t err, nodep;

	if (username != NULL) {
#ifdef USE_SOX
		sox_userdata_t *userdata = (sox_userdata_t*) ud;
		while (1) {
			v = slipstream_receive(buffer);
			if (v > 0) {
				buf_to_pkt(buffer, &pkt);
				if (write_vals(&pkt, &nodep) >= 0) {
					if (csv != NULL)
						write_vals_csv(&nodep);
#ifdef USE_SAGA
					if (db != NULL)
					write_vals_saga(&nodep);
#endif
					if (global_conn != NULL) {
						if (write_vals_sox(&nodep) >= 0) {
							if (global_conn->xmpp_conn->state
									== XMPP_STATE_CONNECTED) {
								if ((err = sox_item_publish_data(global_conn,
										nodes[nodep].item, nodes[nodep].event,
										userdata)) < 0)
									fprintf(
											stderr,
											"ERROR: Could not publish item, error code: %i\n",
											err);
							}
						}
					}
				}
			}
			// Pause for REFRESH µsecond(s)
			usleep(REFRESH);
		}
#endif
	} else {
		while (1) {
			v = slipstream_receive(buffer);
			if (v > 0) {
				buf_to_pkt(buffer, &pkt);
				if (write_vals(&pkt, &nodep) >= 0) {
					if (csv != NULL)
						write_vals_csv(&nodep);
#ifdef USE_SAGA
					if (db != NULL)
					write_vals_saga(&nodep);
#endif
				}
			}
			// Pause for REFRESH µsecond(s)
			usleep(REFRESH);
		}
	}
}

/* Function to print command line usage */
void print_usage(char *prog_name) {
#if defined(USE_SOX) && defined(USE_SAGA)
	fprintf(stdout, "\"%s\" SLIPstream SOX-SAGA-CSV client\n", prog_name);
	fprintf(
			stdout,
			"Usage: %s <-s SLIPstream_server_address> <-port SLIPstream_server_port> <-def default_packet_configuration_file> [-opt detailed_configuration_file] [-u XMPP username -p XMPP password] [-db SAGA_db_location] [-csv CSV_output_file] [-verbose]\n",
			prog_name);
#elif defined(USE_SOX)
	fprintf(stdout, "\"%s\" SLIPstream SOX-CSV client\n", prog_name);
	fprintf(
			stdout,
			"Usage: %s <-s SLIPstream_server_address> <-port SLIPstream_server_port> <-def default_packet_configuration_file> [-opt optional_configuration_file] [-u XMPP username -p XMPP password] [-csv CSV output file] [-verbose]\n",
			prog_name);
#elif defined(USE_SAGA)
	fprintf(stdout, "\"%s\" SLIPstream SAGA-CSV client\n", prog_name);
	fprintf(
			stdout,
			"Usage: %s <-s SLIPstream_server_address> <-port SLIPstream_server_port> <-def default_packet_configuration_file> [-opt optional_configuration_file] [-db SAGA_db_location] [-csv CSV_output_file] [-verbose]\n",
			prog_name);
#else
	fprintf(stdout, "\"%s\" SLIPstream CSV client\n", prog_name);
	fprintf(
			stdout,
			"Usage: %s <-s SLIPstream_server_address> <-port SLIPstream_server_port> <-def default_packet_configuration_file> [-opt optional_configuration_file] [-csv CSV output_file] [-verbose]\n",
			prog_name);
#endif

	fprintf(stdout, "\t-s SLIPstream server IP address or domain name\n");
	fprintf(stdout, "\t-port SLIPstream server port\n");
	fprintf(stdout, "\t-def default packet configuration file\n");
	fprintf(stdout, "\t-opt optional packet configuration file\n");
#ifdef USE_SOX
	fprintf(
			stdout,
			"\t-u username = JID to authenticate (give the full JID, i.e. user@domain)\n");
	fprintf(stdout, "\t-p password = JID user password\n");
#endif
#ifdef USE_SAGA
	fprintf(stdout, "\t-db SAGA database location\n");
#endif
	fprintf(
			stdout,
			"\t-csv CSV file output directory, leave blank for current directory\n");
	fprintf(stdout, "\t-help = print this usage and exit\n");
	fprintf(stdout, "\t-verbose = print info\n");
}

int outlet_actuate(uint32_t mac, uint8_t socket, uint8_t state) {
	int v, size;
	PKT_T my_pkt;
	char buffer[128];

	fprintf(stdout, "actuating\n");
	my_pkt.src_mac = 0x00000000;
	my_pkt.dst_mac = mac;
	my_pkt.type = APP;
	my_pkt.payload[0] = 1; // Number of Elements
	my_pkt.payload[1] = 2; // KEY (2->actuate, 1-> Power Packet)
	my_pkt.payload[2] = socket; // Outlet number 0/1
	my_pkt.payload[3] = state; // Outlet state 0/1
	my_pkt.payload_len = 4;
	size = pkt_to_buf(&my_pkt, buffer);
	v = slipstream_send(buffer, size);
	return v;
}

/* Main */
int main(int argc, char *argv[]) {

#ifdef USE_SOX
	sox_userdata_t userdata;
	sox_conn_t * conn;
#endif

	pthread_t main_loop_thread, saga_writer, slip_ka, user_input;
	uint8_t err = 0;
	uint8_t current_arg_num = 1;
	uint16_t slip_port = 0;
	char *current_arg_name = NULL;
	char *current_arg_val = NULL;
	char *password = NULL;
	char *slip_server = NULL;
	char *db = NULL;

	if (argc == 1) {
		print_usage(argv[0]);
		return -1;
	}

	if (!strcmp(argv[1], "-help")) {
		print_usage(argv[0]);
		return -1;
	}

	while (current_arg_num < argc) {
		current_arg_name = argv[current_arg_num++];

		if (strcmp(current_arg_name, "-help") == 0) {
			print_usage(argv[0]);
			return -1;
		}
		if (strcmp(current_arg_name, "-verbose") == 0) {
			verbose = 1;
			continue;
		}
		if (current_arg_num == argc) {
			print_usage("");
			return -1;
		}

		current_arg_val = argv[current_arg_num++];

		if (strcmp(current_arg_name, "-u") == 0) {
			username = current_arg_val;
		} else if (strcmp(current_arg_name, "-p") == 0) {
			password = current_arg_val;
		} else if (strcmp(current_arg_name, "-s") == 0) {
			slip_server = current_arg_val;
		} else if (strcmp(current_arg_name, "-def") == 0) {
			def = current_arg_val;
		} else if (strcmp(current_arg_name, "-opt") == 0) {
			opt = current_arg_val;
		} else if (strcmp(current_arg_name, "-port") == 0) {
			slip_port = atoi(current_arg_val);
		} else if (strcmp(current_arg_name, "-db") == 0) {
			db = current_arg_val;
		} else if (strcmp(current_arg_name, "-csv") == 0) {
			csv = current_arg_val;
		} else {
			fprintf(stderr, "Unknown argument: %s\n", current_arg_name);
			print_usage(argv[0]);
			return -1;
		}
	}
	if (username == NULL && password != NULL) {
		fprintf(stderr, "Username missing\n");
		print_usage(argv[0]);
		return -1;
	} else if (username != NULL && password == NULL) {
		fprintf(stderr, "Password missing\n");
		print_usage(argv[0]);
		return -1;
	} else if (def == NULL) {
		fprintf(stderr, "Default packet configuration file missing\n");
		print_usage(argv[0]);
		return -1;
	} else if (slip_server == NULL) {
		fprintf(stderr, "SLIPstream server address missing\n");
		print_usage(argv[0]);
		return -1;
	} else if (slip_port == 0) {
		fprintf(stderr, "SLIPstream server port\n");
		print_usage(argv[0]);
		return -1;
	}

#ifdef USE_SOX
	else if (csv == NULL && username == NULL && db == NULL) {
		fprintf(
				stderr,
				"Either SOX, SAGA and/or CSV output functionality is required\n");
		print_usage(argv[0]);
		return -1;
	}
#else
	else if (csv == NULL && db == NULL) {
		fprintf(stderr,
				"Either SAGA and/or CSV output functionality is required\n");
		print_usage(argv[0]);
		return -1;
	}
#endif

#ifdef USE_SAGA
	if (db != NULL)
	init_db(db);
#endif

	slipstream_open(slip_server, slip_port, NONBLOCKING);
	pthread_create(&slip_ka, NULL, slip_keep_alive, NULL);
//pthread_create(&user_input, NULL, user_input_thread, NULL);

#ifdef USE_SOX
	if ((err = read_conf(def, XMLstart_read_conf_default)) >= 0) {
#ifdef USE_SAGA
		pthread_create(&saga_writer, NULL, write_saga, NULL);
#endif

		if (username != NULL) {
			if (verbose == 0)
				conn = sox_connect(username, password,
						(sox_handler_conn) conn_handler, SOX_LEVEL_ERROR,
						&userdata);
			else
				conn = sox_connect(username, password,
						(sox_handler_conn) conn_handler, SOX_LEVEL_DEBUG,
						&userdata);
			sox_run(conn, slip_watcher);
		} else
			slip_watcher(NULL);
		if (userdata.return_value < 0)
			fprintf(stderr, "ERROR: Connection failed\n");
	}
#else
	if ((err = read_conf(def, XMLstart_read_conf_default)) >= 0) {
#ifdef USE_SAGA
		pthread_create(&saga_writer, NULL, write_saga, NULL);
#endif
		slip_watcher(NULL);
	}
#endif

	return err;
}

