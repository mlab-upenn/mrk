#include <stdlib.h>
#include <time.h>
#include <nlist_pkt_handler.h>
#include <sampl.h>
#include <node_cache.h>
#include <globals.h>
#include <ack_pkt.h>
#include <neighbor_pkt.h>
#include <transducer_pkt.h>
#include <transducer_registry.h>
#include <tx_queue.h>
#include <error_log.h>
#include <loc_engine.h>


static char buf[1024];

void loc_extended_nlist_pkt_handler (SAMPL_GATEWAY_PKT_T * gw_pkt)
{
  char node_name[MAX_NODE_LEN];
  char publisher_node_name[MAX_NODE_LEN];
  uint8_t num_msgs, i;
  char timeStr[100],rssiStr[5];
  time_t timestamp;
  int8_t rssi, ret;
  uint8_t t;
  uint32_t tmp;

  if (gw_pkt->payload_len == 0) {
    if (debug_txt_flag)
      printf ("Malformed packet!\n");
    return;
  }


  sprintf (publisher_node_name, "%02x%02x%02x%02x", gw_pkt->subnet_mac[2],
           gw_pkt->subnet_mac[1], gw_pkt->subnet_mac[0], gw_pkt->src_mac);


    printf ("Data for node: %s\n", publisher_node_name);
  // publish XML data for node
  time (&timestamp);
  strftime (timeStr, 100, "%Y-%m-%d %X", localtime (&timestamp));

  g_nlist.num=0;
  sscanf(publisher_node_name,"%X", &tmp );
  g_nlist.mac=tmp;

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
  printf( "  node: %s rssi: %s\n",node_name,rssiStr );
  sscanf(node_name,"%X", &tmp );
  g_nlist.link_mac[g_nlist.num]=tmp;
  g_nlist.rssi[g_nlist.num]=rssi;
  g_nlist.num++;
  }

loc_engine_update(&g_nlist);





}
