#ifndef BINARY_BLOB_H 
#define BINARY_BLOB_H 

#include <globals.h>
#include <sampl.h>
#include "transducer_registry.h"
#include <transducer_pkt.h>

typedef struct binary_blob_tran_pkt 
{
	uint8_t size; 
	uint8_t *data;   
} BINARY_BLOB_TRAN_PKT;



uint8_t binary_blob_tran_pack( TRANSDUCER_MSG_T *t, BINARY_BLOB_TRAN_PKT *p);
uint8_t binary_blob_tran_unpack( TRANSDUCER_MSG_T *t, BINARY_BLOB_TRAN_PKT *p);

#endif
