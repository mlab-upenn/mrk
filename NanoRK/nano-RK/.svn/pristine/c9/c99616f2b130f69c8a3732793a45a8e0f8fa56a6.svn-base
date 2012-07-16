#ifndef _XMPP_PROXY_H_
#define _XMPP_PROXY_H_

#include <loudmouth/loudmouth.h>
#include <expat.h>
#include <stdint.h>
#include <soxlib.h>


#define MAX_PROXY_CONS	10

typedef struct proxy_connections {
  XMPPConnection *connection;
  char src_jid[128];
  uint32_t mac_addr;
  uint8_t active;
  uint8_t timeout;
} PROXY_CON_T;

PROXY_CON_T proxy_con[MAX_PROXY_CONS];


char p_server[128], p_pubsub[128], p_ssl_fingerprint[128];

void proxy_configure(char *xmpp_server,uint32_t xmpp_server_port, char *pubsub_server, char *xmpp_ssl_fingerprint );
static void proxy_msg_handler(LmMessage *m);
void proxy_cleanup();
void proxy_login_and_send(uint32_t mac_addr, char *src_jid, char *passwd, char *dst_jid, char *msg, uint8_t len,uint8_t txt_mode, uint16_t timeout );
void proxy_login_and_publish(uint32_t mac_addr, char *src_jid, char *passwd, char *event_node, char *msg, uint8_t len, uint8_t txt_mode, uint16_t timeout);
uint32_t proxy_find_mac_addr(char *jid);
#endif
