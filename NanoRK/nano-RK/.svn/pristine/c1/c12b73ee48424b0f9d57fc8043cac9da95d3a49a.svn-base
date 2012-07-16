#include <stdlib.h>
#include <time.h>
#include "xmpp_pkt_writer.h"
#include <sampl.h>
#include <soxlib.h>
#include <node_cache.h>
#include <globals.h>
#include <ping_pkt.h>
#include <xmpp_pkt.h>
#include <ack_pkt.h>
#include <neighbor_pkt.h>
#include <transducer_pkt.h>
#include <transducer_registry.h>
#include <xmpp_proxy.h>
#include <tx_queue.h>
#include <error_log.h>


void xmpp_pkt_handler (SAMPL_GATEWAY_PKT_T * gw_pkt)
{
  XMPP_PKT_T p;
  char node_name[MAX_NODE_LEN];
  char full_jid[128];
  char full_dst_jid[128];
  int8_t ret, i;
  uint32_t src_mac_addr;
  uint8_t x;

  if (gw_pkt->payload_len == 0) {
    if (debug_txt_flag)
      printf ("Malformed packet!\n");
    log_write ("Malformed XMPP-lite msg from network ");
    return;
  }


  xmpp_pkt_unpack (&p, gw_pkt->payload, 0);
  sprintf (node_name, "%02x%02x%02x%02x", (uint8_t) gw_pkt->subnet_mac[2],
           (uint8_t) gw_pkt->subnet_mac[1], (uint8_t) gw_pkt->subnet_mac[0],
           (uint8_t) gw_pkt->src_mac);

  printf ("XMPP msg:\n");
  printf ("  mac-addr=%s\n", node_name);
  if (p.explicit_src_jid_flag == 1)
    printf ("  src-jid[%d]=%s\n", p.src_jid_size, p.src_jid);
  printf ("  passwd=%s\n", p.passwd);
  printf ("  dst-jid[%d]=%s\n", p.dst_jid_size, p.dst_jid);
  printf ("  timeout=%d\n", p.timeout);
  printf ("  explicit_src_jid=%d\n", p.explicit_src_jid_flag);
  printf ("  binary_flag=%d\n", p.binary_flag);
  printf ("  pubsub_flag=%d\n", p.pub_sub_flag);
  printf ("  msg-size=%d\n", p.msg_size);
  if (p.binary_flag == 0)
    printf ("  ascii_msg=%s", p.msg);
  else {
    printf ("  binary_msg=");
    for (i = 0; i < p.msg_size; i++)
      printf ("%02x", (uint8_t) p.msg[i]);
  }
  printf ("\n");
  src_mac_addr = (gw_pkt->subnet_mac[2] << 24) |
    (gw_pkt->subnet_mac[1] << 16) |
    (gw_pkt->subnet_mac[0] << 8) | gw_pkt->src_mac;


  if (p.explicit_src_jid_flag == 0)
    sprintf (full_jid, "%s@%s", node_name, p_server);
  else {
    if (strstr (p.src_jid, "@") == NULL)
      sprintf (full_jid, "%s@%s", p.src_jid, p_server);
    else
      sprintf (full_jid, "%s", p.src_jid);

  }

  if (p.dst_jid_size == 0) {
    printf ("Error in proxy: No destination jid!\n");
    log_write ("XMPP Proxy failed to find dst jid");
    return;
  }
  if (p.pub_sub_flag == 1) {
    // dst_jid is the event node you wish to publish too
    proxy_login_and_publish (src_mac_addr, full_jid, p.passwd, p.dst_jid,
                             p.msg, p.msg_size, p.binary_flag, p.timeout);
  }
  else {
    if (strstr (p.dst_jid, "@") == NULL)
      sprintf (full_dst_jid, "%s@%s", p.dst_jid, p_server);
    else
      sprintf (full_dst_jid, "%s", p.dst_jid);
    proxy_login_and_send (src_mac_addr, full_jid, p.passwd, full_dst_jid,
                          p.msg, p.msg_size, p.binary_flag, p.timeout);
  }
}
