#ifndef LOC_DATA_H 
#define LOC_DATA_H 

#include <globals.h>
#include <sampl.h>
#include "transducer_registry.h"
#include <transducer_pkt.h>

#define MAX_TRAN_NEIGHBORS	6

#define UNKOWN		0
#define ITX_PKT		1


#define NEIGHBOR_SIZE	5

typedef struct neighbor_t 
{
	int8_t rssi; 
	uint8_t mac[4];   
} NEIGHBOR_T;



typedef struct nlist_tran_pkt 
{
	uint8_t num_neighbors; 
	NEIGHBOR_T neighbor[MAX_TRAN_NEIGHBORS];   
} NLIST_TRAN_PKT;


uint8_t nlist_tran_pack( TRANSDUCER_MSG_T *t, NLIST_TRAN_PKT *p );
uint8_t nlist_tran_unpack( TRANSDUCER_MSG_T *t, NLIST_TRAN_PKT *p );

#endif
