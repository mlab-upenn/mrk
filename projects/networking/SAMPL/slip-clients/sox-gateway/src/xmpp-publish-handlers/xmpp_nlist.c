#include <stdlib.h>
#include <time.h>
#include <xmpp_nlist.h>
#include <sampl.h>
#include <soxlib.h>
#include <node_cache.h>
#include <globals.h>
#include <ack_pkt.h>
#include <neighbor_pkt.h>
#include <transducer_pkt.h>
#include <transducer_registry.h>
#include <xmpp_proxy.h>
#include <tx_queue.h>
#include <error_log.h>


static char buf[1024];

void extended_nlist_pkt_handler (SAMPL_GATEWAY_PKT_T * gw_pkt)
{
  char node_name[MAX_NODE_LEN];
  char publisher_node_name[MAX_NODE_LEN];
  uint8_t num_msgs, i;
  char timeStr[100],rssiStr[5];
  time_t timestamp;
  int8_t rssi, ret;
  uint8_t t;
  SOXMessage *msg = NULL;

  if (gw_pkt->payload_len == 0) {
    if (debug_txt_flag)
      printf ("Malformed packet!\n");
    return;
  }


  sprintf (publisher_node_name, "%02x%02x%02x%02x", gw_pkt->subnet_mac[2],
           gw_pkt->subnet_mac[1], gw_pkt->subnet_mac[0], gw_pkt->src_mac);


  if (debug_txt_flag == 1)
    printf ("Data for node: %s\n", publisher_node_name);
  // Check nodes and add them if need be
  if (xmpp_flag == 1)
    check_and_create_node (publisher_node_name);
  // publish XML data for node
  time (&timestamp);
  strftime (timeStr, 100, "%Y-%m-%d %X", localtime (&timestamp));

  msg = create_sox_message();
  msg_add_device_installation(msg,publisher_node_name,publisher_node_name,"FIREFLY","A Firefly Node",timeStr);
  

  num_msgs = gw_pkt->payload[0];
  if(debug_txt_flag)
  	printf ("Extended Neighbor List for node: %s\n", publisher_node_name);
  if (gw_pkt->num_msgs > (MAX_PKT_PAYLOAD / NLIST_PKT_SIZE))
    return;
  for (i = 0; i < num_msgs; i++) {
    sprintf (node_name, "%02x%02x%02x%02x", gw_pkt->payload[1 + i * 5 + 3],
             gw_pkt->payload[1 + i * 5 + 2],
             gw_pkt->payload[1 + i * 5 + 1], gw_pkt->payload[1 + i * 5 + 0]);
    rssi = (int8_t) gw_pkt->payload[1 + i * 5 + 4];
    sprintf( rssiStr,"%d",rssi );
    msg_add_device_connection(msg,publisher_node_name,"unknown",node_name,rssiStr);
  if(debug_txt_flag) printf( "  node: %s rssi: %s\n",node_name,rssiStr );
  }


  ret = publish_sox_message (connection, publisher_node_name, msg);
  delete_sox_message(msg);
  if (ret != XMPP_NO_ERROR) {
    sprintf (global_error_msg, "XMPP Error Extended nList: %s", ERROR_MESSAGE (ret));
    log_write (global_error_msg);
    sprintf (global_error_msg, "-> event_node=%s",publisher_node_name);
    log_write (global_error_msg);
    return -1;
  }



//  if (debug_txt_flag == 1)
//    printf ("Publish: %s\n", buf);
//  if (xmpp_flag == 1)
//    ret = publish_to_node (connection, publisher_node_name, buf);
//  if (xmpp_flag && ret != XMPP_NO_ERROR)
//    printf ("XMPP Error: %s\n", ERROR_MESSAGE (ret));


}
