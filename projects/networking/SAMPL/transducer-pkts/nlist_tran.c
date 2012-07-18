#include <nlist_tran.h>
#include <transducer_registry.h>
#include <transducer_pkt.h>
#include <stdio.h>

uint8_t nlist_tran_pack(TRANSDUCER_MSG_T *t , NLIST_TRAN_PKT * p)
{
uint8_t i;
if(p->num_neighbors>MAX_TRAN_NEIGHBORS) p->num_neighbors=MAX_TRAN_NEIGHBORS;
	t->payload[0]=p->num_neighbors;
for(i=0; i< p->num_neighbors; i++ )
{
	t->payload[1+(i*NEIGHBOR_SIZE)]=p->neighbor[i].mac[3];
	t->payload[2+(i*NEIGHBOR_SIZE)]=p->neighbor[i].mac[2];
	t->payload[3+(i*NEIGHBOR_SIZE)]=p->neighbor[i].mac[1];
	t->payload[4+(i*NEIGHBOR_SIZE)]=p->neighbor[i].mac[0];
	t->payload[5+(i*NEIGHBOR_SIZE)]=p->neighbor[i].rssi;
}

  return ((p->num_neighbors*NEIGHBOR_SIZE)+1);
}


uint8_t nlist_tran_unpack(TRANSDUCER_MSG_T *t,
                                    NLIST_TRAN_PKT *p)
{
uint8_t i;
  p->num_neighbors=t->payload[0];
  if(p->num_neighbors>MAX_TRAN_NEIGHBORS) p->num_neighbors=MAX_TRAN_NEIGHBORS;
  for(i=0; i<p->num_neighbors; i++ )
  {
	p->neighbor[i].mac[3]=t->payload[1+(i*NEIGHBOR_SIZE)];
	p->neighbor[i].mac[2]=t->payload[2+(i*NEIGHBOR_SIZE)];
	p->neighbor[i].mac[1]=t->payload[3+(i*NEIGHBOR_SIZE)];
	p->neighbor[i].mac[0]=t->payload[4+(i*NEIGHBOR_SIZE)];
	p->neighbor[i].rssi=t->payload[5+(i*NEIGHBOR_SIZE)];
  } 
  return ((p->num_neighbors*NEIGHBOR_SIZE)+1);
}

