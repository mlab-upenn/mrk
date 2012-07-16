#include <globals.h>
#include <nrk.h>
#include <sampl.h>
#include <transducer_handler.h>
#include <transducer_registry.h>


// This is a callback function that gets executed when a transducer
// packet addressed to this node arrives.  Messages packed into out_msg
// are then returned.
int8_t transducer_handler(TRANSDUCER_MSG_T *in_msg, TRANSDUCER_MSG_T *out_msg)
{
uint8_t i;

nrk_kprintf( PSTR("in_msg\r\n  mac_addr=") ); printf( "%d\r\n", in_msg->mac_addr );
nrk_kprintf( PSTR("  type=") ); printf( "%d\r\n", in_msg->type);
nrk_kprintf( PSTR("  len=") ); printf( "%d\r\n", in_msg->len);
nrk_kprintf( PSTR("  payload= [ " ));
for(i=0; i<in_msg->len; i++ )
	printf( "%d ",in_msg->payload[i] );
nrk_kprintf( PSTR( "]\r\n" ));


// By default nothing will be returned because you don't want to NCK back to packets for other devices
// Make sure to return 0 if you did not add a packet and 1 if you did.

// Return ACK if good and no other reply
out_msg->type=TRAN_ACK;
out_msg->len=0;
return 1;


// Return NCK if bad and no other reply
//out_msg->type=TRAN_NCK;
//out_msg->len=0;
// return 1;


return 0;
}
