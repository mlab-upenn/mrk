#include <composer_handler.h>
#include <stdint.h>
#include <error_log.h>
#include <sampl.h>
#include <transducer_pkt.h>
#include <transducer_registry.h>
#include <globals.h>
#include <tx_queue.h>
#include <ff_basic_sensor_pkt.h>
#include <transducer_pkt.h>
#include <ff_power.h>
#include <transducer_registry.h>


#define HPR_ACTUATE	0
#define HPR_SOCKET_0	1
#define HPR_SOCKET_1	2
#define HPR_READ	3
#define HPR_DEBUG	4

void handle_power_request()
{
int i;
int cmd,len;
int arg_a,arg_b,arg_c;
uint32_t tmp;
uint8_t subnet[3],dst_mac;
SAMPL_PEER_2_PEER_PKT_T 	p2p_pkt;
FF_POWER_ACTUATE_PKT  		ff_pwr_actuate;
FF_POWER_RQST_PKT     		ff_pwr_rqst;
TRANSDUCER_PKT_T      		tran_pkt;
TRANSDUCER_MSG_T      		tran_msg;
char tx_buf[150];
char mbuf[128];
char buf[128];
RETRY_PARAMS_T retry;

cmd=0;
//printf( "Handle Power for node: %s\n",xmpp_in_event_node );
for(i=0; i<xmpp_in_num_params; i++ )
{
//printf( "  name: %s value: %s\n",&xmpp_in_attr_name[i], &xmpp_in_attr_value[i] );
if(strcmp( &xmpp_in_attr_name[i], "name" )==0) 
	{
	if(strcmp( &xmpp_in_attr_value[i], "socket0" )==0) 
		{
		cmd=HPR_ACTUATE;
		arg_a=HPR_SOCKET_0;
		}
	if(strcmp( &xmpp_in_attr_value[i], "socket1" )==0) 
		{
		cmd=HPR_ACTUATE;
		arg_a=HPR_SOCKET_1;
		}
	if(strcmp( &xmpp_in_attr_value[i], "read" )==0) 
		cmd=HPR_READ;
	if(strcmp( &xmpp_in_attr_value[i], "debug" )==0) 
		cmd=HPR_DEBUG;
	}

if(strcmp( &xmpp_in_attr_name[i], "value" )==0) 
	{
	if(strcmp( &xmpp_in_attr_value[i], "on" )==0) arg_b=1;
	if(strcmp( &xmpp_in_attr_value[i], "off" )==0) arg_b=0;
	if(strcmp( &xmpp_in_attr_value[i], "socket0" )==0) arg_a=0;
	if(strcmp( &xmpp_in_attr_value[i], "socket1" )==0) arg_a=1;
	}
}

 sscanf( xmpp_in_event_node,"%x",&tmp );
 subnet[2]=(tmp&0xff000000) >> 24;
 subnet[1]=(tmp&0xff0000) >> 16;
 subnet[0]=(tmp&0xff00) >> 8;
 dst_mac =(tmp & 0xff);

 	p2p_pkt.payload_len=0;
        p2p_pkt.buf=tx_buf;
        p2p_pkt.buf_len=P2P_PAYLOAD_START;
        p2p_pkt.payload_start=P2P_PAYLOAD_START;
        p2p_pkt.payload=&(tx_buf[P2P_PAYLOAD_START]);

        // These are parameters that can be adjusted for different packets
        p2p_pkt.pkt_type=TRANSDUCER_PKT;
        p2p_pkt.ctrl_flags= MOBILE_MASK | LINK_ACK | DEBUG_FLAG | ENCRYPT;
        p2p_pkt.seq_num=0; // Use the gateway's spiffy auto-cnt when set to 0
        p2p_pkt.priority=0;
        p2p_pkt.hop_cnt=0;
        p2p_pkt.ttl=5;
        p2p_pkt.ack_retry=10;
        p2p_pkt.src_subnet_mac[0]=0;
        p2p_pkt.src_subnet_mac[1]=0;
        p2p_pkt.src_subnet_mac[2]=0;
        p2p_pkt.src_mac=0;
        p2p_pkt.last_hop_mac=0;
        p2p_pkt.next_hop_mac=BROADCAST;
        p2p_pkt.dst_subnet_mac[0] = subnet[0];
        p2p_pkt.dst_subnet_mac[1] = subnet[1];
        p2p_pkt.dst_subnet_mac[2] = subnet[2];
        p2p_pkt.dst_mac = dst_mac;
        p2p_pkt.check_rate=100;  // B-mac check rate in ms

  	tran_pkt.num_msgs=0;
        tran_pkt.checksum=0;
        tran_pkt.msgs_payload=buf;

	tran_msg.mac_addr = dst_mac;
        tran_msg.type = TRAN_POWER_PKT;
        tran_msg.len = 0;
        tran_msg.payload = mbuf;

if(cmd==HPR_ACTUATE)
{
	ff_pwr_actuate.socket0_state=SOCKET_HOLD;
        ff_pwr_actuate.socket1_state=SOCKET_HOLD;
	if(arg_a==HPR_SOCKET_0)
	{
		if( arg_b==1 ) ff_pwr_actuate.socket0_state=SOCKET_ON;
		if( arg_b==0 ) ff_pwr_actuate.socket0_state=SOCKET_OFF;
	
	}
       	if(arg_a==HPR_SOCKET_1)
	{
		if( arg_b==1 ) ff_pwr_actuate.socket1_state=SOCKET_ON;
		if( arg_b==0 ) ff_pwr_actuate.socket1_state=SOCKET_OFF;
	
	}
         // Pack application specifc message into transducer message
        tran_msg.len=ff_power_actuate_pack(mbuf, &ff_pwr_actuate);
}

if(cmd==HPR_DEBUG || cmd==HPR_READ)
{
	ff_pwr_rqst.socket=arg_a;
        ff_pwr_rqst.pkt_type=DEBUG_PKT;
         // Pack application specifc message into transducer message
        tran_msg.len=ff_power_rqst_pack(mbuf, &ff_pwr_rqst);
}

        // Add the transducer message to the transducer packet
        len=transducer_msg_add( &tran_pkt, &tran_msg);

 p2p_pkt.payload_len =
      transducer_pkt_pack(&tran_pkt,p2p_pkt.payload);
         // This takes the structure and packs it into the raw
        // array that is sent using SLIP
        pack_peer_2_peer_packet( &p2p_pkt);

        // set low-level auto-retries
	retry.cnt=3;
	retry.timeout=4;
	retry.reply_mac=dst_mac;
	// Add to queue with higher than 0 priority
        tx_q_add(p2p_pkt.buf,p2p_pkt.buf_len,1, &retry);


}





