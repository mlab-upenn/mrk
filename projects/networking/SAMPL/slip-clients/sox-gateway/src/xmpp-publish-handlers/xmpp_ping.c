#include <stdlib.h>
#include <time.h>
#include <xmpp_ping.h>
#include <sampl.h>
#include <soxlib.h>
#include <node_cache.h>
#include <globals.h>
#include <ping_pkt.h>
#include <ack_pkt.h>
#include <xmpp_proxy.h>
#include <tx_queue.h>


static char buf[1024];

void send_xmpp_ping_pkt (SAMPL_GATEWAY_PKT_T * gw_pkt)
{
  PING_PKT_T p;
  char node_name[MAX_NODE_LEN];
  char timeStr[64];
  time_t timestamp;
  int i, ret;
  uint8_t t;

  return;

  if (gw_pkt->payload_len == 0 || gw_pkt->payload_len > MAX_PAYLOAD) {
    if (debug_txt_flag)
      printf ("Malformed packet!\n");
    return;
  }

  if (gw_pkt->num_msgs > (MAX_PKT_PAYLOAD / PING_PKT_SIZE))
    return;
  for (i = 0; i < gw_pkt->num_msgs; i++) {
    t = ping_pkt_get (&p, gw_pkt->payload, i);
    if (t == 0)
      break;
    sprintf (node_name, "%02x%02x%02x%02x", gw_pkt->subnet_mac[2],
             gw_pkt->subnet_mac[1], gw_pkt->subnet_mac[0], p.mac_addr);


    if (debug_txt_flag == 1)
      printf ("Data for node: %s\n", node_name);
    // Check nodes and add them if need be
    if (xmpp_flag == 1)
      check_and_create_node (node_name);
    // publish XML data for node
    time (&timestamp);
    strftime (timeStr, 100, "%Y-%m-%d %X", localtime (&timestamp));
    sprintf (buf, "<Node id=\"%s\" type=\"FIREFLY\" timestamp=\"%s\"></Node>",
             node_name, timeStr);

    if (debug_txt_flag == 1)
      printf ("Publish: %s\n", buf);
    if (xmpp_flag == 1)
      ret = publish_to_node (connection, node_name, buf);
    if (xmpp_flag == 1 && ret != XMPP_NO_ERROR)
      printf ("XMPP Error: %s\n", ERROR_MESSAGE (ret));


  }

}
