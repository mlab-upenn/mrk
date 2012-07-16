#include <binary_blob.h>
#include <transducer_registry.h>
#include <transducer_pkt.h>
#include <stdio.h>

uint8_t binary_blob_tran_pack( TRANSDUCER_MSG_T *t, BINARY_BLOB_TRAN_PKT *p)
{
uint8_t i;

for(i=0; i< p->size; i++ )
	t->payload[i]=p->data[i];

  return (p->size);
}


uint8_t binary_blob_tran_unpack( TRANSDUCER_MSG_T *t, BINARY_BLOB_TRAN_PKT *p)
{
uint8_t i;

p->size=t->len;
for(i=0; i< p->size; i++ )
	p->data[i]=t->payload[i];

return (p->size);
}

